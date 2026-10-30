#!/usr/bin/env python3
"""cam_ctl.py —— 板载相机的**全部参数**人为调节工具（主机侧，ssh 操作板子）。

为什么要有它
------------
相机插在板子上（`/dev/video0`），参数只能用 `v4l2-ctl` 在板上改。手工敲一长串
`--set-ctrl=` 容易漏项、也没法对比；更麻烦的是**改参数必须在流停掉时做**，否则
ffmpeg 会输出纯色帧（std≈0），看起来像"没生效/更糟"（踩过）。

这个工具把三件事做掉：
1. **枚举**相机全部控件（名字/类型/范围/步长/当前值/菜单项）；
2. **设置**（自动先停流 → 设参数 → 让预览自动重连，避免纯色帧）；
3. **对比**（`sweep` 一次扫一串值各抓一帧；`web` 给网页滑杆 + 实时画面）。

用法
----
    python3 core/board/tools/cam_ctl.py list
    python3 core/board/tools/cam_ctl.py get exposure_absolute gain
    python3 core/board/tools/cam_ctl.py set exposure_absolute=10 brightness=0 gamma=90
    python3 core/board/tools/cam_ctl.py reset            # 回到记录下来的可用值
    python3 core/board/tools/cam_ctl.py capture /tmp/a.jpg
    python3 core/board/tools/cam_ctl.py preset save good
    python3 core/board/tools/cam_ctl.py preset load good
    python3 core/board/tools/cam_ctl.py sweep exposure_absolute 10,20,40,80 --out /tmp/sw
    python3 core/board/tools/cam_ctl.py web --port 8510   # 浏览器 http://127.0.0.1:8510/

注意：`--restart-stream`（默认开）会先杀掉板上正在读相机的 ffmpeg；如果你正开着
`cam_preview.py`，它会在 1.5 s 内自动重连，不用手动重启。
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PRESET_DIR = ROOT / "data/fpga/cam_presets"
DEFAULT_KEY = "data/fpga/board_ssh/board_key"
DEFAULT_BOARD = "root@192.168.137.242"
DEV = "/dev/video0"

#: 记录下来的"能用"的一组（2026-10-23：深色页面 + 当前光线，mean≈48 / std≈38）
KNOWN_GOOD = {
    "exposure_auto": 1,
    "exposure_auto_priority": 0,
    "exposure_absolute": 10,
    "gain": 4,
    "brightness": 0,
    "contrast": 64,
    "saturation": 64,
    "gamma": 90,
    "sharpness": 2,
    "backlight_compensation": 0,
    "power_line_frequency": 1,
    "white_balance_temperature_auto": 0,
    "white_balance_temperature": 4000,
}

#: 抓单帧用的 ffmpeg 参数（和 start_cam.sh 一致；缺 `-input_format mjpeg` 会卡住）
FFMPEG_ONE = (
    "ffmpeg -hide_banner -loglevel error -y -fflags nobuffer -flags low_delay "
    "-probesize 32 -analyzeduration 0 -f v4l2 -input_format mjpeg "
    "-video_size 640x480 -i {dev} -frames:v 1 -q:v 2 {out}"
)


# ---------------------------------------------------------------------------
# ssh / 板端命令
# ---------------------------------------------------------------------------
class Board:
    def __init__(self, target: str, key: str, dev: str = "auto") -> None:
        self.target = target
        self.key = key
        self.dev = dev

    def run(self, cmd: str, timeout: float = 30.0) -> str:
        argv = ["ssh", "-i", self.key,
                "-o", "StrictHostKeyChecking=no",
                "-o", "UserKnownHostsFile=/dev/null",
                "-o", "LogLevel=ERROR",
                "-o", "ConnectTimeout=8",
                self.target, cmd]
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return (p.stdout or "") + (p.stderr or "")

    # -- 设备节点 -----------------------------------------------------------
    def auto_dev(self) -> str:
        """自动找**USB 相机**的采集节点（跳过板载 platform:mvx 视频设备）。

        为什么要自动：相机被拔插后枚举号会变（实测从 `/dev/video0` 变成
        `/dev/video1`），而旧进程还抓着 `/dev/video0 (deleted)` 的 fd 继续出帧
        —— 表现是"画面在动但明显不对、参数改了没反应"。写死节点就一定会再踩。
        """
        out = self.run("v4l2-ctl --list-devices 2>&1")
        group_ok = False
        cands: list[str] = []
        for line in out.splitlines():
            if line and not line.startswith((" ", "\t")) and line.rstrip().endswith(":"):
                group_ok = "platform" not in line and "mvx" not in line
                continue
            m = re.search(r"(/dev/video\d+)", line)
            if m and group_ok:
                cands.append(m.group(1))
        #: 优先选支持 MJPG 的那个（UVC 相机里它是采集节点，另一个是 metadata）
        for dev in cands:
            fmts = self.run(f"v4l2-ctl -d {dev} --list-formats 2>&1")
            if "MJPG" in fmts.upper():
                return dev
        return cands[0] if cands else DEV

    def resolve_dev(self, want: str = "auto") -> str:
        if want and want != "auto":
            return want
        d = self.auto_dev()
        if d != DEV:
            print(f"（相机自动探测：{d}）", file=sys.stderr)
        return d

    # -- 相机 ---------------------------------------------------------------
    def list_controls(self) -> list[dict]:
        out = self.run(f"v4l2-ctl -d {self.dev} -L 2>&1")
        ctrls: list[dict] = []
        cur: dict | None = None
        for raw in out.splitlines():
            line = raw.rstrip()
            if not line.strip():
                continue
            m = re.match(r"\s*(\w+)\s+0x[0-9a-fA-F]+\s+\((\w+)\)\s*:(.*)$", line)
            if m:
                name, typ, rest = m.group(1), m.group(2), m.group(3)
                cur = {"name": name, "type": typ, "min": None, "max": None,
                       "step": None, "default": None, "value": None,
                       "menu": {}}
                for key in ("min", "max", "step", "default", "value"):
                    mm = re.search(rf"{key}=(-?\d+)", rest)
                    if mm:
                        cur[key] = int(mm.group(1))
                if typ in ("int", "int64", "bool"):
                    mm = re.search(r"value=(-?\d+)", rest)
                    if mm:
                        cur["value"] = int(mm.group(1))
                if typ == "menu":
                    mm = re.search(r"value=(-?\d+)", rest)
                    if mm:
                        cur["value"] = int(mm.group(1))
                ctrls.append(cur)
                continue
            #: `-L` 的菜单行形如： `\t\t  0: Auto Exposure`
            mm = re.match(r"\s*(\d+):\s*(.+?)\s*$", line)
            if cur is not None and mm and cur.get("type") == "menu":
                cur["menu"][int(mm.group(1))] = mm.group(2)
        return ctrls

    def stream_pids(self) -> list[int]:
        out = self.run('pgrep -f "^ffmpeg" || true')
        return [int(x) for x in out.split() if x.strip().isdigit()]

    def stop_stream(self) -> int:
        out = self.run('for p in $(pgrep -f "^ffmpeg"); do kill -TERM $p; done; echo stopped')
        return len(self.stream_pids())

    def set_ctrl(self, items: dict) -> str:
        pairs = ",".join(f"{k}={int(v)}" for k, v in items.items())
        return self.run(f"v4l2-ctl -d {self.dev} --set-ctrl={pairs} 2>&1")

    def capture(self, out_path: str) -> bool:
        self.run(FFMPEG_ONE.format(dev=self.dev, out=out_path), timeout=40.0)
        return bool(self.run(f"ls -l {out_path} 2>/dev/null | head -1").strip())


# ---------------------------------------------------------------------------
# 打印
# ---------------------------------------------------------------------------
def show(ctrls: list[dict]) -> None:
    print(f"{'name':34s} {'type':6s} {'value':>7s} {'min':>6s} {'max':>6s} "
          f"{'step':>4s}  menu")
    for c in ctrls:
        menu = " ".join(f"{k}={v}" for k, v in sorted(c["menu"].items()))[:44]
        print(f"{c['name']:34s} {c['type']:6s} {str(c['value']):>7s} "
              f"{str(c['min']):>6s} {str(c['max']):>6s} {str(c['step']):>4s}  {menu}")


def apply_with_restart(board: Board, items: dict, restart: bool) -> int:
    """设参数。**先停流再设**，否则流会输出纯色帧（std≈0）。"""
    had_stream = bool(board.stream_pids())
    if had_stream and restart:
        board.stop_stream()
        time.sleep(0.8)
    msg = board.set_ctrl(items).strip()
    if msg:
        print(msg)
    #: 校验回读
    names = ",".join(items)
    got = board.run(f"v4l2-ctl -d {DEV} --get-ctrl={names} 2>&1")
    print(got.strip())
    bad = []
    for k, v in items.items():
        m = re.search(rf"{k}:\s*(-?\d+)", got)
        if not m or int(m.group(1)) != int(v):
            bad.append(k)
    if bad:
        print(f"⚠ 这些没生效（会被范围/依赖裁剪）：{bad}")
        return 2
    if had_stream and restart:
        print("（流已停，预览会在 ~1.5 s 后自动重连；也可以用 `capture` 抓一帧确认）")
    return 0


# ---------------------------------------------------------------------------
# 网页版
# ---------------------------------------------------------------------------
PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>camera controls</title>
<style>
body{{background:#111;color:#ddd;font:13px/1.45 monospace;margin:0}}
#wrap{{display:flex;gap:16px;padding:10px}}
#left{{flex:0 0 660px}}
#right{{flex:1}}
#snap{{width:640px;max-width:100%;border:1px solid #333;image-rendering:pixelated}}
table{{border-collapse:collapse;width:100%}}
td{{padding:2px 4px;border-bottom:1px solid #222;vertical-align:middle}}
input[type=number],select{{width:96px;background:#1b1b1b;color:#eee;border:1px solid #333}}
input[type=range]{{width:150px;vertical-align:middle}}
.name{{color:#8cf}}
.val{{color:#fff;text-align:right}}
button{{background:#234;color:#dff;border:1px solid #356;padding:4px 10px;cursor:pointer}}
#msg{{color:#9f9;padding:4px 10px;white-space:pre-wrap}}
#st{{color:#888;padding:4px 10px}}
</style></head><body>
<div id="wrap">
  <div id="left"><img id="snap" src="/snap.jpg?t=0"><div id="st">…</div></div>
  <div id="right">
    <form method="post" action="/apply" id="f"><table>{rows}</table>
      <p><button type="submit">应用（会先停流再设，预览自动重连）</button>
         <button type="submit" name="action" value="reset">恢复记录值</button></p>
    </form>
    <div id="msg">{msg}</div>
    <p><a style="color:#8cf" href="/ctrl.json">/ctrl.json</a> ·
       <a style="color:#8cf" href="/snap.jpg">/snap.jpg</a></p>
  </div>
</div>
<script>
let n=0,f=0;
function tick(){{
  const t=Date.now();
  const im=document.getElementById('snap');
  im.onload=()=>{{n++;document.getElementById('st').textContent='帧 '+n+' 失败 '+f;}};
  im.onerror=()=>{{f++;document.getElementById('st').textContent='帧 '+n+' 失败 '+f;}};
  im.src='/snap.jpg?t='+t;
  setTimeout(tick,500);
}}
tick();
</script></body></html>
"""


def rows_html(ctrls: list[dict]) -> str:
    out = []
    for c in ctrls:
        name = c["name"]
        cur = "" if c["value"] is None else c["value"]
        if c["type"] == "menu" and c["menu"]:
            opts = "".join(
                f'<option value="{k}"{" selected" if c["value"] == k else ""}>'
                f'{k} — {v}</option>' for k, v in sorted(c["menu"].items()))
            widget = f'<select name="{name}">{opts}</select>'
        elif c["max"] is not None and c["min"] is not None and c["max"] > c["min"]:
            step = c["step"] or 1
            widget = (f'<input type="range" min="{c["min"]}" max="{c["max"]}" '
                      f'step="{step}" value="{cur}" '
                      f'oninput="this.nextElementSibling.value=this.value">'
                      f'<input type="number" name="{name}" min="{c["min"]}" '
                      f'max="{c["max"]}" step="{step}" value="{cur}">')
        else:
            widget = f'<input type="number" name="{name}" value="{cur}">'
        out.append(f'<tr><td class="name">{name}</td>'
                   f'<td class="val">{cur}</td>'
                   f'<td>{widget}</td>'
                   f'<td style="color:#666">{c["min"]}~{c["max"]} /{c["step"]}</td></tr>')
    return "\n".join(out)


def make_handler(board: Board, restart: bool):
    state = {"msg": ""}

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, body: bytes, ctype: str, code: int = 200) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):  # noqa: N802
            p = self.path.split("?", 1)[0]
            if p in ("/", "/index.html"):
                ctrls = board.list_controls()
                html = PAGE.format(rows=rows_html(ctrls), msg=state["msg"])
                self._send(html.encode("utf-8"), "text/html; charset=utf-8")
                return
            if p == "/ctrl.json":
                self._send(json.dumps(board.list_controls(), ensure_ascii=False,
                                      indent=1).encode(), "application/json")
                return
            if p in ("/snap.jpg", "/snap"):
                jpg = proxy_snap()
                if jpg:
                    self._send(jpg, "image/jpeg")
                else:
                    self._send(b"no frame", "text/plain", 503)
                return
            self._send(b"404", "text/plain", 404)

        def do_POST(self):  # noqa: N802
            n = int(self.headers.get("Content-Length") or 0)
            form = urllib.parse.parse_qs(self.rfile.read(n).decode("utf-8", "ignore"))
            if form.get("action", [""])[0] == "reset":
                items = dict(KNOWN_GOOD)
            else:
                items = {}
                known = {c["name"]: c for c in board.list_controls()}
                for k, v in form.items():
                    if k in ("action",) or k not in known:
                        continue
                    try:
                        items[k] = int(v[0])
                    except (ValueError, IndexError):
                        continue
            rc = apply_with_restart(board, items, restart)
            state["msg"] = ("已应用：" + " ".join(f"{k}={v}" for k, v in items.items())
                            if rc == 0 else f"⚠ 有项没生效（rc={rc}），见控制台/回读")
            self.send_response(303)
            self.send_header("Location", "/")
            self.send_header("Content-Length", "0")
            self.end_headers()

    return H


def proxy_snap() -> bytes | None:
    """优先用正在跑的 cam_preview（8500）的 /snap.jpg；没有就自己抓一帧转发。"""
    try:
        with urllib.request.urlopen("http://127.0.0.1:8500/snap.jpg",
                                    timeout=3.0) as r:
            if r.status == 200:
                return r.read()
    except OSError:
        pass
    try:
        with urllib.request.urlopen("http://127.0.0.1:8501/snap.jpg",
                                    timeout=3.0) as r:
            if r.status == 200:
                return r.read()
    except OSError:
        pass
    return None


# ---------------------------------------------------------------------------
def parse_pairs(args: list[str]) -> dict:
    out: dict[str, int] = {}
    for a in args:
        if "=" not in a:
            print(f"⚠ 忽略（要 name=value）：{a}", file=sys.stderr)
            continue
        k, v = a.split("=", 1)
        out[k.strip()] = int(v)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="板载相机全部参数的人为调节工具")
    ap.add_argument("--board", default=DEFAULT_BOARD)
    ap.add_argument("--key", default=str(ROOT / DEFAULT_KEY))
    ap.add_argument("--dev", default="auto",
                    help="相机设备节点；auto=自动探测 USB 相机（默认，重插也不会错）")
    ap.add_argument("--no-restart", action="store_true",
                    help="改参数时不先停流（不推荐：会得到纯色帧）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("list", help="列出相机所有控件")
    g = sub.add_parser("get", help="读若干控件当前值")
    g.add_argument("names", nargs="*")
    s = sub.add_parser("set", help="设置控件 name=value …")
    s.add_argument("items", nargs="+")
    sub.add_parser("reset", help="恢复到记录下来的可用值")
    c = sub.add_parser("capture", help="抓一帧到主机路径")
    c.add_argument("path", nargs="?", default="/tmp/cam_shot.jpg")
    c.add_argument("--local", default="", help="同时拷回主机这个路径")
    p = sub.add_parser("preset", help="预设：save/load NAME")
    p.add_argument("op", choices=("save", "load", "ls"))
    p.add_argument("name", nargs="?", default="")
    sw = sub.add_parser("sweep", help="一次扫一串值各抓一帧（做对比图）")
    sw.add_argument("ctrl")
    sw.add_argument("values")
    sw.add_argument("--out", default="/tmp/cam_sweep")
    w = sub.add_parser("web", help="网页版（滑杆 + 实时画面）")
    w.add_argument("--port", type=int, default=8510)

    a = ap.parse_args(argv)
    board = Board(a.board, a.key)
    board.dev = board.resolve_dev(a.dev)
    restart = not a.no_restart

    if a.cmd == "list":
        ctrls = board.list_controls()
        if not ctrls:
            print("⚠ 没读到控件：板子连不上，或 /dev/video0 被占/不存在")
            return 1
        show(ctrls)
        return 0

    if a.cmd == "get":
        names = ",".join(a.names) if a.names else ""
        print(board.run(f"v4l2-ctl -d {DEV} --get-ctrl={names} 2>&1").strip())
        return 0

    if a.cmd == "set":
        return apply_with_restart(board, parse_pairs(a.items), restart)

    if a.cmd == "reset":
        return apply_with_restart(board, dict(KNOWN_GOOD), restart)

    if a.cmd == "capture":
        remote = "/tmp/cam_ctl_shot.jpg"
        if not board.capture(remote):
            print("⚠ 抓帧失败（相机被占/板子不可达）")
            return 1
        dst = a.local or a.path
        subprocess.run(["scp", "-i", a.key, "-o", "StrictHostKeyChecking=no",
                        "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR",
                        f"{a.board}:{remote}", dst], check=False)
        print(f"已抓帧：{dst}")
        return 0

    if a.cmd == "preset":
        PRESET_DIR.mkdir(parents=True, exist_ok=True)
        if a.op == "ls":
            for f in sorted(PRESET_DIR.glob("*.json")):
                print(f.name)
            return 0
        if not a.name:
            print("要指定名字")
            return 2
        f = PRESET_DIR / f"{a.name}.json"
        if a.op == "save":
            cur = {c["name"]: c["value"] for c in board.list_controls()
                   if c["value"] is not None}
            f.write_text(json.dumps(cur, ensure_ascii=False, indent=1))
            print(f"已保存 {f}（{len(cur)} 项）")
            return 0
        if not f.exists():
            print(f"没有这个预设：{f}")
            return 2
        items = {k: int(v) for k, v in json.loads(f.read_text()).items()}
        return apply_with_restart(board, items, restart)

    if a.cmd == "sweep":
        outdir = Path(a.out)
        outdir.mkdir(parents=True, exist_ok=True)
        base = {c["name"]: c["value"] for c in board.list_controls()
                if c["value"] is not None}
        made = []
        for v in [x.strip() for x in a.values.split(",") if x.strip()]:
            items = dict(base)
            items[a.ctrl] = int(v)
            print(f"— {a.ctrl}={v}")
            apply_with_restart(board, items, restart)
            remote = "/tmp/cam_ctl_sweep.jpg"
            if not board.capture(remote):
                print("  ⚠ 抓帧失败")
                continue
            dst = outdir / f"{a.ctrl}_{v}.jpg"
            subprocess.run(["scp", "-i", a.key, "-o", "StrictHostKeyChecking=no",
                            "-o", "UserKnownHostsFile=/dev/null", "-o", "LogLevel=ERROR",
                            f"{a.board}:{remote}", str(dst)], check=False)
            made.append(str(dst))
        print("抓好的图：")
        for m in made:
            print("  ", m)
        return 0

    if a.cmd == "web":
        ctrls = board.list_controls()
        if not ctrls:
            print("⚠ 读不到控件（板子不可达/相机不在）")
            return 1
        httpd = ThreadingHTTPServer(("127.0.0.1", int(a.port)),
                                   make_handler(board, restart))
        httpd.daemon_threads = True
        print(f"相机参数面板： http://127.0.0.1:{int(a.port)}/  "
              f"（{len(ctrls)} 个控件；改完点“应用”）", flush=True)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
