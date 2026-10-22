#!/usr/bin/env python3
"""extract_sd_image.py —— 从**整盘镜像**里只读地抠文件（不需要 root、不需要挂载）。

用途：板子要刷成别的系统之前，把"重建运行环境"需要的东西从备份镜像里取出来：
`/etc` 的关键配置、`/root/tools`、`/root/board_src`、`/lib/firmware`、FAT 引导区
（BOOT.BIN / BL31.ELF / *.BIT），以及 `dpkg` 包清单。

为什么自己写解析而不用 mount：
* 容器里 `CapEff=0`、`sudo` 被 no_new_privs 禁掉，`mount -o loop` 需要 root；
* 板上虽然能挂，但当前板子可能不在线（这次就是探针把总线挂住、板子失联）。

支持：ext4（extent，深度任意）/ FAT16（含子目录、长文件名），只读。
不支持：ext4 的 encrypted/inline_data、FAT32 的 LFN 交叉条目（会用 8.3 短名兜底）。

    python3 extract_sd_image.py IMAGE --list-parts
    python3 extract_sd_image.py IMAGE --part 2 --out DIR /etc /root/tools ...
    python3 extract_sd_image.py IMAGE --part 1 --out DIR --fat /   # FAT 引导区整盘
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
from pathlib import Path

SECTOR = 512


# --------------------------------------------------------------------------
# MBR
# --------------------------------------------------------------------------
def parts_of(img: Path) -> list[tuple[int, int, int]]:
    """返回 [(序号, 起始字节, 字节长度)]（只解析 MBR 主分区表）。"""
    with open(img, "rb") as f:
        f.seek(0)                      # MBR 在整盘镜像的偏移 0（ImageUSB 容器才在 512）
        mbr = f.read(SECTOR)
    if mbr[510:512] != b"\x55\xaa":
        raise SystemExit("偏移 0 处不是 MBR（若这是 ImageUSB 容器，先去掉 512 字节头再跑）")
    out = []
    for i in range(4):
        e = mbr[446 + 16 * i: 462 + 16 * i]
        typ = e[4]
        lba, cnt = struct.unpack_from("<II", e, 8)
        if cnt and typ:
            out.append((i + 1, lba * SECTOR, cnt * SECTOR, typ))
    return out


# --------------------------------------------------------------------------
# ext4
# --------------------------------------------------------------------------
class Ext4:
    def __init__(self, img: Path, base: int):
        self.img, self.base = img, base
        self._f = open(img, "rb")
        sb = self.rd(base + 1024, 1024)
        if struct.unpack_from("<H", sb, 0x38)[0] != 0xEF53:
            raise SystemExit("该分区不是 ext4")
        self.inode_size = struct.unpack_from("<H", sb, 0x58)[0] or 128
        self.inodes_per_g = struct.unpack_from("<I", sb, 0x28)[0]
        self.first_data = struct.unpack_from("<I", sb, 0x14)[0]
        self.log_bs = struct.unpack_from("<I", sb, 0x18)[0]
        self.bs = 1024 << self.log_bs
        self.desc_size = struct.unpack_from("<H", sb, 0xFE)[0] or 32
        self.incompat = struct.unpack_from("<I", sb, 0x60)[0]
        self.is64 = bool(self.incompat & 0x80)
        self.gdt = self.base + (self.first_data + 1) * self.bs

    def rd(self, off: int, n: int) -> bytes:
        self._f.seek(off)
        return self._f.read(n)

    def inode(self, ino: int):
        g, idx = divmod(ino - 1, self.inodes_per_g)
        d = self.rd(self.gdt + g * self.desc_size, self.desc_size)
        itab = struct.unpack_from("<I", d, 0x8)[0]
        if self.is64:
            itab |= struct.unpack_from("<I", d, 0x28)[0] << 32
        raw = self.rd(self.base + itab * self.bs + idx * self.inode_size, self.inode_size)
        mode = struct.unpack_from("<H", raw, 0)[0]
        size = struct.unpack_from("<I", raw, 4)[0]
        if self.is64:
            size |= struct.unpack_from("<I", raw, 0x6C)[0] << 32
        flags = struct.unpack_from("<I", raw, 0x20)[0]
        return mode, size, flags, raw

    def blocks(self, raw: bytes) -> list[int]:
        out: list[int] = []
        ib = raw[0x28:0x28 + 60]
        if struct.unpack_from("<H", ib, 0)[0] != 0xF30A:
            # 非 extent：小文件用 inode 内直接块（≤12 个）兜底
            for k in range(12):
                b = struct.unpack_from("<I", ib, 4 * k)[0]
                if b:
                    out.append(b)
            return out
        entries, _maxe, depth = struct.unpack_from("<HHH", ib, 2)

        def walk(node: bytes, n: int, dep: int) -> None:
            for i in range(n):
                e = node[12 + 12 * i: 24 + 12 * i]
                if len(e) < 12:
                    return
                if dep == 0:
                    _lb, ln, hi, lo = struct.unpack_from("<IHHI", e, 0)
                    blk = lo | (hi << 16)
                    out.extend(blk + k for k in range(ln))
                else:
                    _lb, lo, hi = struct.unpack_from("<IIH", e, 0)
                    child = self.rd(self.base + (lo | (hi << 16)) * self.bs, self.bs)
                    if len(child) >= 12:
                        walk(child, struct.unpack_from("<H", child, 2)[0], dep - 1)

        walk(ib, entries, depth)
        return out

    def read_file(self, ino: int, data: bytes) -> bytes:
        """把文件内容写回 data（inode 已含大小）。"""
        return data

    def get(self, ino: int) -> bytes:
        mode, size, _flags, raw = self.inode(ino)
        if (mode & 0xF000) == 0xA000:                      # 符号链接
            if size < 60 and struct.unpack_from("<H", raw, 0x28)[0] != 0xF30A:
                return raw[0x28:0x28 + size]
            return b""
        buf = bytearray()
        for b in self.blocks(raw):
            if len(buf) >= size:
                break
            buf += self.rd(self.base + b * self.bs, self.bs)
        return bytes(buf[:size])

    def ls(self, ino: int) -> dict[str, int]:
        data = self.get(ino)
        out, off = {}, 0
        while off + 8 <= len(data):
            i, rec, nl = struct.unpack_from("<IHB", data, off)
            if rec < 8:
                break
            if i and nl:
                out[data[off + 8:off + 8 + nl].decode("latin1")] = i
            off += rec
        return out

    def lookup(self, path: str) -> int | None:
        ino = 2
        for p in [x for x in path.split("/") if x]:
            d = self.ls(ino)
            if p not in d:
                return None
            ino = d[p]
        return ino


def walk_ext4(fs: Ext4, ino: int, dst: Path, rel: str, manifest: list, skip: tuple[str, ...]) -> None:
    mode, _size, _f, _raw = fs.inode(ino)
    kind = mode & 0xF000
    if kind == 0x4000:                                    # 目录
        dst.mkdir(parents=True, exist_ok=True)
        for name, child in fs.ls(ino).items():
            if name in (".", "..") or name in skip:
                continue
            walk_ext4(fs, child, dst / name, f"{rel}/{name}", manifest, skip)
    elif kind == 0x8000:                                  # 普通文件
        blob = fs.get(ino)
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(blob)
        manifest.append((rel, len(blob)))
    elif kind == 0xA000:                                  # 符号链接：记下来但不解析
        tgt = fs.get(ino).decode("latin1", "replace")
        manifest.append((rel, -1))
        (dst.parent / (dst.name + ".symlink")).write_text(tgt, encoding="utf-8")


# --------------------------------------------------------------------------
# FAT16
# --------------------------------------------------------------------------
class Fat16:
    def __init__(self, img: Path, base: int):
        self.img, self.base = img, base
        self._f = open(img, "rb")
        b = self.rd(0, 512)
        self.bps = struct.unpack_from("<H", b, 0x0B)[0]
        self.spc = b[0x0D]
        self.rsvd = struct.unpack_from("<H", b, 0x0E)[0]
        self.nfats = b[0x10]
        self.rootent = struct.unpack_from("<H", b, 0x11)[0]
        self.fatsz = struct.unpack_from("<H", b, 0x16)[0] or struct.unpack_from("<I", b, 0x24)[0]
        self.root_off = (self.rsvd + self.nfats * self.fatsz) * self.bps
        self.data_off = self.root_off + self.rootent * 32

    def rd(self, off: int, n: int) -> bytes:
        self._f.seek(self.base + off)
        return self._f.read(n)

    def fat(self, cl: int) -> int:
        v = self.rd(self.rsvd * self.bps + cl * 2, 2)
        return struct.unpack("<H", v)[0] if len(v) == 2 else 0xFFFF

    def chain(self, cl: int) -> list[int]:
        out = []
        while 2 <= cl < 0xFFF8 and len(out) < 1_000_000:
            out.append(cl)
            cl = self.fat(cl)
        return out

    def cluster_data(self, cl: int) -> bytes:
        return self.rd(self.data_off + (cl - 2) * self.spc * self.bps, self.spc * self.bps)

    def entries(self, raw: bytes) -> list[tuple[str, int, int, int]]:
        """[(名字, 属性, 首簇, 大小)]，处理 LFN。"""
        out, lfn = [], ""
        for i in range(0, len(raw) - 31, 32):
            e = raw[i:i + 32]
            if e[0] == 0:
                break
            if e[0] == 0xE5:
                continue
            attr = e[11]
            if attr == 0x0F:                              # LFN
                part = (e[1:11] + e[14:26] + e[28:32]).decode("utf-16-le", "ignore")
                lfn = part.replace("\x00", "").replace("\uffff", "") + lfn
                continue
            name = e[0:8].decode("latin1").strip()
            ext = e[8:11].decode("latin1").strip()
            short = name + ("." + ext if ext else "")
            full = lfn.strip() or short
            lfn = ""
            cl = struct.unpack_from("<H", e, 26)[0]
            size = struct.unpack_from("<I", e, 28)[0]
            out.append((full, attr, cl, size))
        return out

    def read_file(self, cl: int, size: int) -> bytes:
        return b"".join(self.cluster_data(c) for c in self.chain(cl))[:size]


def walk_fat(fs: Fat16, entries, dst: Path, rel: str, manifest: list) -> None:
    for name, attr, cl, size in entries:
        if name in (".", ".."):
            continue
        p = dst / name
        r = f"{rel}/{name}"
        if attr & 0x10:                                   # 目录
            p.mkdir(parents=True, exist_ok=True)
            raw = b"".join(fs.cluster_data(c) for c in fs.chain(cl))
            walk_fat(fs, fs.entries(raw), p, r, manifest)
        elif size > 0:
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(fs.read_file(cl, size))
            manifest.append((r, size))


# --------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--part", type=int, default=2, help="分区序号（1 起）")
    ap.add_argument("--out", default="", help="输出目录")
    ap.add_argument("--list-parts", action="store_true")
    ap.add_argument("--fat", action="store_true", help="按 FAT 解析整个分区")
    ap.add_argument("--skip", default="CMakeFiles,__pycache__,.git",
                    help="递归时跳过的目录名（逗号分隔）")
    ap.add_argument("--paths", nargs="*", default=[],
                    help="ext4: 要导出的路径（以 / 开头）")
    a = ap.parse_args()
    img = Path(a.image)
    parts = parts_of(img)
    if a.list_parts:
        for n, off, ln, typ in parts:
            print(f"  p{n}: type=0x{typ:02x} offset={off} size={ln} ({ln/2**30:.2f} GiB)")
        return 0
    sel = [p for p in parts if p[0] == a.part]
    if not sel:
        raise SystemExit(f"没有第 {a.part} 个分区")
    _, off, _ln, _typ = sel[0]
    out = Path(a.out or f"sd_extract_p{a.part}")
    out.mkdir(parents=True, exist_ok=True)
    manifest: list[tuple[str, int]] = []
    if a.fat:
        fs = Fat16(img, off)
        walk_fat(fs, fs.entries(fs.rd(fs.root_off, fs.rootent * 32)), out, "", manifest)
    else:
        fs = Ext4(img, off)
        skip = tuple(s for s in a.skip.split(",") if s)
        for p in (a.paths or ["/"]):
            ino = fs.lookup(p)
            if ino is None:
                print(f"  ✗ 镜像里没有 {p}")
                continue
            walk_ext4(fs, ino, out / p.strip("/"), p, manifest, skip)
    manifest.sort()
    (out / "MANIFEST.txt").write_text(
        "\n".join(f"{s:>12d}  {r}" for r, s in manifest) + "\n", encoding="utf-8")
    total = sum(s for _, s in manifest if s > 0)
    print(f"导出 {len(manifest)} 项，共 {total/2**20:.1f} MiB → {out}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
