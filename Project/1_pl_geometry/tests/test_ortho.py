"""vision_ortho 的回归：与定点 oracle 逐字节一致 + 测试台自证。

为什么要有这一条（2026-10-25）
------------------------------
正射 RTL 调试时，光"手工跑一次 tb"是不够的：这一阶段**三次**被测试台自身的 bug 带偏
（`+dbg` 被当成 h0、按上一拍地址喂数、负数系数被当 plusarg 跳过）。所以这里放两条用例：

1. `test_ortho_bit_exact`：真实相机帧 → RTL 输出 vs `tools/ortho_fixed_ref` 的定点 oracle，
   **逐字节比**。系数由 oracle 的 `VR_DBG_HQ` 打印、tb 直接吃，**禁止手抄**（手抄会被二次量化）。
2. `test_tb_selftest_constant_image`：喂一张**纯色**图，输出必须整幅同色 —— 任何采样点取到的
   都是同一个值，所以只要通路（地址→数据→输出）错拍/错位就会露出来。这条用来证明"量具是准的"。

依赖：Verilator（见 core/fpga/ref/README.md 的免 root 装法）、g++、cv2（读 jpg 相机快照）。
缺任何一样就 skip，不把环境问题算成回归失败。
"""
import pathlib
import shutil
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
COSIM = ROOT / "core" / "fpga" / "cosim"
RTL = ROOT / "core" / "fpga" / "rtl"
REF = ROOT / "core" / "fpga" / "ref"
TOOLS = ROOT / "core" / "fpga" / "tools"
SKY = ROOT / "data" / "fpga" / "quad.json"

VERILATOR_ROOT = pathlib.Path("/home/zhang/.local/verilator-5.020")


def _verilator() -> str:
    v = VERILATOR_ROOT / "bin" / "verilator"
    if v.exists():
        return str(v)
    found = shutil.which("verilator")
    if found:
        return found
    pytest.skip("没有 Verilator")


def _env() -> dict:
    import os
    e = dict(os.environ)
    e["VERILATOR_ROOT"] = str(VERILATOR_ROOT)
    e["PATH"] = f"{VERILATOR_ROOT / 'bin'}:{e.get('PATH', '')}"
    return e


@pytest.fixture(scope="module")
def exe(tmp_path_factory) -> pathlib.Path:
    """把 vision_ortho + tb_ortho 编成可执行（放在 tmp，不污染仓库）。"""
    v = _verilator()
    out = tmp_path_factory.mktemp("obj_ortho")
    exe = out / "Vvision_ortho"
    r = subprocess.run(
        [v, "--cc", "--exe", "--build", "-j", "4", "-O2", "-Wno-fatal",
         "--top-module", "vision_ortho", f"-I{RTL}", "--Mdir", str(out),
         str(RTL / "vision_ortho.v"), str(RTL / "vision_div_pipe.v"),
         str(COSIM / "tb_ortho.cpp"), "-o", "Vvision_ortho"],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM))
    assert r.returncode == 0, r.stdout + r.stderr
    assert exe.exists()
    return exe


@pytest.fixture(scope="module")
def oracle() -> pathlib.Path:
    """编译定点 oracle（ortho_fixed_ref）。"""
    if not (TOOLS / "ortho_fixed_ref.cpp").exists() or not SKY.exists():
        pytest.skip("缺 ortho_fixed_ref.cpp 或 quad.json")
    out = COSIM / "obj_ortho"
    out.mkdir(exist_ok=True)
    exe = out / "ortho_fixed_ref"
    r = subprocess.run(
        ["g++", "-std=c++17", "-O2", f"-I{ROOT / 'core' / 'board'}",
         "-o", str(exe), str(TOOLS / "ortho_fixed_ref.cpp")],
        capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    return exe


def _camera_frame(tmp: pathlib.Path) -> pathlib.Path:
    """取一张真实相机帧（640×480 RGB24 → PPM）。

    优先用**入库**的 `data/fpga/ortho_stim/*.ppm`（就是手工验证时用的那两帧）——
    用例和手工验证必须吃同一份输入，否则"手工 0 不符、用例红"这种假失败会反复出现。
    没有入库素材时才退回从 `camera_last.jpg` 现取（缺 cv2/缺快照就 skip）。
    """
    stim = ROOT / "data" / "fpga" / "ortho_stim"
    stored = sorted(stim.glob("*.ppm")) if stim.is_dir() else []
    if stored:
        return stored[0]
    try:
        import cv2  # noqa: F401
    except Exception:
        pytest.skip("没有 cv2，取不到相机快照")
    import cv2
    import numpy as np
    cands = sorted((ROOT / "data" / "fpga" / "loop_logs").glob("*/camera_last.jpg"))
    if not cands:
        pytest.skip("没有 data/fpga/loop_logs/*/camera_last.jpg")
    im = cv2.imread(str(cands[-1]))
    if im is None:
        pytest.skip(f"读不了 {cands[-1]}")
    im = cv2.resize(im, (640, 480))
    raw = cv2.cvtColor(im, cv2.COLOR_BGR2RGB).tobytes()
    p = tmp / "cam.ppm"
    p.write_bytes(b"P6\n640 480\n255\n" + raw)
    return p


def _stored_frames() -> list:
    """入库的相机帧（`data/fpga/ortho_stim/*.ppm`）：用例与手工验证吃同一份输入。"""
    stim = ROOT / "data" / "fpga" / "ortho_stim"
    return sorted(stim.glob("*.ppm")) if stim.is_dir() else []


def _raw_of(ppm: pathlib.Path, tmp: pathlib.Path) -> pathlib.Path:
    """oracle 的 stdin 是**裸 RGB**（没有 PPM 头）；tb 的 argv 是 PPM。
       两者混用会让 oracle 把 15 字节的头当像素 —— 实测差 45 万字节。"""
    b = ppm.read_bytes()
    i = 0
    nl = 0
    while nl < 3 and i < len(b):
        if b[i:i + 1] == b"\n":
            nl += 1
        i += 1
    raw = tmp / (ppm.stem + ".raw")
    raw.write_bytes(b[i:])
    return raw


def test_ortho_bit_exact(exe, oracle, tmp_path):
    cam = _camera_frame(tmp_path)
    cam_raw = _raw_of(cam, tmp_path)
    exp_pre = str(tmp_path / "exp")
    hq = subprocess.run([str(oracle), "--quad", str(SKY), "--dump", exp_pre],
                        stdin=open(cam_raw, "rb"), capture_output=True, text=True,
                        env={**_env(), "VR_DBG_HQ": "1"})
    assert hq.returncode == 0, hq.stderr
    coefs = hq.stdout.strip().split()[0:13]
    assert len(coefs) == 13, hq.stdout
    exp = tmp_path / "exp_00001.ppm"
    assert exp.exists(), "oracle 没 dump 出 PPM"

    r = subprocess.run([str(exe), str(cam), str(exp), *coefs],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout


@pytest.fixture(scope="module")
def exe_sys(tmp_path_factory) -> pathlib.Path:
    """P0 全链路：DDR 模型 → AXI 读主口 → 行缓存 → 正射（vision_ortho_sys）。"""
    v = _verilator()
    out = tmp_path_factory.mktemp("obj_sys")
    exe = out / "Vvision_ortho_sys"
    r = subprocess.run(
        [v, "--cc", "--exe", "--build", "-j", "4", "-O2", "-Wno-fatal",
         "--top-module", "vision_ortho_sys", f"-I{RTL}", "--Mdir", str(out),
         str(RTL / "vision_ortho_sys.v"), str(RTL / "vision_ortho.v"),
         str(RTL / "vision_linecache.v"), str(RTL / "vision_linefill.v"),
         str(RTL / "vision_axi_master.v"), str(RTL / "vision_div_pipe.v"),
         str(COSIM / "tb_ortho_sys.cpp"), "-o", "Vvision_ortho_sys"],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM))
    assert r.returncode == 0, r.stdout + r.stderr
    assert exe.exists()
    return exe


@pytest.mark.parametrize("cam", _stored_frames(), ids=[p.name for p in _stored_frames()])
def test_ortho_sys_bit_exact(exe_sys, oracle, cam, tmp_path):
    """全链路逐字节对 oracle。

    为什么这条不能省（2026-10-25 实测）：`vision_linecache` 的读口把 24 位字拼成了
    反序（byte0 落到 [7:0]），而 `tb_linecache` 自己按同样的反序检查 —— **单测 72/72 全过**。
    直到这条"对着已验证的消费者（正射）比"的用例才抓到（表现：B/R 颠倒、最大差 16）。
    单测编码了和 RTL 同一个错，等于什么也没证明；交叉检查才是门禁。
    """
    cam_raw = _raw_of(cam, tmp_path)
    exp_pre = str(tmp_path / "exp")
    hq = subprocess.run([str(oracle), "--quad", str(SKY), "--dump", exp_pre],
                        stdin=open(cam_raw, "rb"), capture_output=True, text=True,
                        env={**_env(), "VR_DBG_HQ": "1"})
    assert hq.returncode == 0, hq.stderr
    coefs = hq.stdout.strip().split()[0:13]
    assert len(coefs) == 13, hq.stdout
    exp = tmp_path / "exp_00001.ppm"
    assert exp.exists(), "oracle 没 dump 出 PPM"

    r = subprocess.run([str(exe_sys), str(cam), str(exp), *coefs],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout            # 页面逐字节
    assert "数据不符 0" in r.stdout, r.stdout        # AXI 读回的字
    assert "PASS" in r.stdout, r.stdout


@pytest.fixture(scope="module")
def exe_divp(tmp_path_factory) -> pathlib.Path:
    """流水线除法器单测（与 C 的截断整除对拍）。它有自己的一张门禁：正射的结果一旦
    被"商算错"污染，逐字节对拍会报**整幅错**，但定位不到是除法器还是别处 —— 所以先单独验。"""
    v = _verilator()
    out = tmp_path_factory.mktemp("obj_divp")
    exe = out / "Vvision_div_pipe"
    r = subprocess.run(
        [v, "--cc", "--exe", "--build", "-j", "4", "-O2", "-Wno-fatal",
         "--top-module", "vision_div_pipe", f"-I{RTL}", "--Mdir", str(out),
         str(RTL / "vision_div_pipe.v"), str(COSIM / "tb_divpipe.cpp"),
         "-o", "Vvision_div_pipe"],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM))
    assert r.returncode == 0, r.stdout + r.stderr
    return exe


def test_divpipe_matches_c(exe_divp):
    """II=1 的流水线除法器：20 万对随机 + 边界（商 0 / 商 2^34-1 / 商超宽）零不符。"""
    r = subprocess.run([str(exe_divp), "200000"], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "商不符 0" in r.stdout and "溢出标志不符 0" in r.stdout, r.stdout
    assert "PASS" in r.stdout, r.stdout


@pytest.fixture(scope="module")
def exe_ca(tmp_path_factory) -> pathlib.Path:
    """相机域适配（vision_camadapt）：与 tools/camadapt_ref.py 逐字节对拍。"""
    v = _verilator()
    out = tmp_path_factory.mktemp("obj_ca")
    exe = out / "Vvision_camadapt"
    r = subprocess.run(
        [v, "--cc", "--exe", "--build", "-j", "4", "-O2", "-Wno-fatal",
         "--top-module", "vision_camadapt", f"-I{RTL}", "--Mdir", str(out),
         str(RTL / "vision_camadapt.v"), str(COSIM / "tb_camadapt.cpp"),
         "-o", "Vvision_camadapt"],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM))
    assert r.returncode == 0, r.stdout + r.stderr
    return exe


@pytest.mark.parametrize("cam", _stored_frames(), ids=[p.name for p in _stored_frames()])
def test_camadapt_bit_exact(exe_ca, oracle, cam, tmp_path):
    """相机域适配必须与 pl_vision.cpp:403 那段（照抄成 camadapt_ref.py）逐字节一致。

    为什么这条是硬门禁：适配后的颜色就是 PL 墙判据的**输入**（R−B>40 算墙、亮度>195 算角色），
    错一个像素就可能漏报一道墙。实测踩过：窗口差一格（只排除最老一格、应排除两格），
    36 个像素被误标成墙、最大差正好 = TINT(60)。
    """
    cam_raw = _raw_of(cam, tmp_path)
    exp_pre = str(tmp_path / "exp")
    hq = subprocess.run([str(oracle), "--quad", str(SKY), "--dump", exp_pre],
                        stdin=open(cam_raw, "rb"), capture_output=True, text=True,
                        env={**_env(), "VR_DBG_HQ": "1"})
    assert hq.returncode == 0, hq.stderr
    coefs = hq.stdout.strip().split()[0:13]
    page = tmp_path / "exp_00001.ppm"          # 正射 oracle 的页面图（= RTL 正交的输出）
    assert page.exists()
    adapt = tmp_path / "adapt.ppm"
    r = subprocess.run([sys.executable, str(TOOLS / "camadapt_ref.py"), str(page), str(adapt)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    r = subprocess.run([str(exe_ca), str(page), str(adapt)], capture_output=True, text=True,
                       timeout=600)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
    assert "PASS" in r.stdout, r.stdout


@pytest.fixture(scope="module")
def exe_p0(tmp_path_factory) -> pathlib.Path:
    """P0 本地集成 cosim（DDR→寄存器→正射→vision_top→几何帧）。"""
    v = _verilator()
    out = tmp_path_factory.mktemp("obj_p0")
    exe = out / "Vvision_axi_p0"
    ref = subprocess.run(["gcc", "-O2", "-std=c11", f"-I{REF}", "-DVR_NO_MAIN", "-c",
                          str(REF / "vision_ref.c"), "-o", str(out / "vision_ref.o")],
                         capture_output=True, text=True)
    assert ref.returncode == 0, ref.stdout + ref.stderr
    rtl = [str(RTL / f) for f in (
        "vision_axi.v", "vision_pixfeed.v", "vision_top.v", "vision_ccl.v", "vision_div.v",
        "vision_ortho.v", "vision_linecache.v", "vision_linefill.v",
        "vision_axi_master.v", "vision_ortho_sys.v", "vision_div_pipe.v")]
    r = subprocess.run(
        [v, "--cc", "--exe", "--build", "-j", "4", "-O2", "-Wno-fatal",
         "--top-module", "vision_axi", f"-I{RTL}", "--Mdir", str(out),
         "-CFLAGS", f"-I{REF}", *rtl, str(COSIM / "tb_p0.cpp"),
         str(out / "vision_ref.o"), "-o", "Vvision_axi_p0"],
        capture_output=True, text=True, env=_env(), cwd=str(COSIM))
    assert r.returncode == 0, r.stdout + r.stderr
    return exe


@pytest.mark.parametrize("cam", _stored_frames(), ids=[p.name for p in _stored_frames()])
def test_p0_integration_bit_exact(exe_p0, oracle, cam, tmp_path):
    """P0 全链路（正射在 PL 里 + vision_top）与 C 参考逐字节一致。

    这是**集成层**的门禁：`tb_ortho`/`tb_ortho_sys` 只验到"正射逐字节对"，
    从没验过"正射 → vision_top"的握手。实测就是在这里抓到的真 bug：
    `out_sof` 被做成"每行行首"，而 `vision_top` 的 `din_sof` 是**帧开始**（收到就 wr_row<=0）
    → 每行都被打回第 0 行、永远收不满（state 一直停在 S_RECV）。
    """
    page = tmp_path / "page.ppm"
    cam_raw = _raw_of(cam, tmp_path)
    hq = subprocess.run([str(oracle), "--quad", str(SKY), "--dump", str(tmp_path / "exp")],
                        stdin=open(cam_raw, "rb"), capture_output=True, text=True,
                        env={**_env(), "VR_DBG_HQ": "1"})
    assert hq.returncode == 0, hq.stderr
    page = tmp_path / "exp_00001.ppm"
    assert page.exists()
    #: 系数**从 oracle 的 VR_DBG_HQ 打印里取**，再喂给 tb_p0（禁止 tb 内手抄）：
    #: 一旦 quad.json 变了，手抄的那组就是旧 quad 的，门禁会红得毫无意义。
    coefs = [v for v in hq.stdout.strip().split() if v.lstrip("-").isdigit()][0:13]
    assert len(coefs) == 13, hq.stdout
    r = subprocess.run([str(exe_p0), str(cam), str(page), *coefs, "--max-cycles", "3000000"],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "几何帧逐字节：不符 0" in r.stdout, r.stdout
    assert "PASS" in r.stdout, r.stdout


def test_tb_selftest_constant_image(exe, oracle, tmp_path):
    """纯色图：任何采样点都一样，所以输出必须整幅同色（验通路，不看几何）。"""
    p = tmp_path / "flat.ppm"
    p.write_bytes(b"P6\n640 480\n255\n" + bytes([90, 90, 90]) * (640 * 480))
    p_raw = _raw_of(p, tmp_path)
    exp_pre = str(tmp_path / "flat_exp")
    hq = subprocess.run([str(oracle), "--quad", str(SKY), "--dump", exp_pre],
                        stdin=open(p_raw, "rb"), capture_output=True, text=True,
                        env={**_env(), "VR_DBG_HQ": "1"})
    coefs = hq.stdout.strip().split()[0:13]
    exp = tmp_path / "flat_exp_00001.ppm"
    r = subprocess.run([str(exe), str(p), str(exp), *coefs],
                       capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "不符 0" in r.stdout, r.stdout
