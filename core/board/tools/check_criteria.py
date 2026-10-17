#!/usr/bin/env python
"""判据 ③ 的**算法层**验证：在合成场景（有真值）上量缺口中心/墙带 y 的误差。

用法：Env/bin/python core/board/tools/check_criteria.py
说明：真实相机素材需要干净画面；相机损坏时先用这个确认几何链本身没问题。"""
import importlib.util, subprocess, tempfile, pathlib
from PIL import Image
spec=importlib.util.spec_from_file_location("t","core/board/tests/test_band_gap_units_cpp.py")
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
tmp=pathlib.Path(tempfile.mkdtemp())
exe=tmp/"bg"
subprocess.run(["g++","-std=c++17","-O2","-I","core/board","-o",str(exe),
                "core/board/tools/band_gap_probe.cpp"],check=True,capture_output=True)
import re
conds=[("baseline",{}),("dark",{"gain":0.55,"offset":-8.0}),("bright",{"gain":1.6,"offset":40.0}),
       ("rot6",{"rot_deg":6.0}),("edge_wall",{"wall_y":479.0})]
print(f"{'条件':<12}{'标记':>5}{'门':>4}{'缺口中心':>9}{'真值':>6}{'误差':>6}{'墙带y':>7}{'真值':>6}{'误差':>6}")
ok_right=0
for name,kw in conds:
    rgb,_to,size=m.scene(**kw); ppm=tmp/f"{name}.ppm"
    ppm.write_bytes(b"P6\n%d %d\n255\n"%size+rgb)
    out=subprocess.run([str(exe),str(ppm)],capture_output=True,text=True).stdout
    # 标记数（用 mtest 逻辑：临时编译一次也行，这里用 band_gap 的门结果）
    d=re.search(r"found=(\d).*?y0=(-?\d+) y1=(-?\d+) x0=(-?\d+) x1=(-?\d+)", out)
    e=re.search(r"extent ok=(\d) x=(-?\d+)\.\.(-?\d+) y=(-?\d+)\.\.(-?\d+)", out)
    fx,fy,fw_,fh_ = m.field_rect_in_image()
    s=640.0/(int(e.group(3))-int(e.group(2))+1)
    cx=(0.5*(int(d.group(4))+int(d.group(5)))-int(e.group(2)))*s
    cy=(0.5*(int(d.group(2))+int(d.group(3)))-fy)*(480.0/fh_)
    wall_true=kw.get("wall_y",300.0)
    ecx=cx-289.0; ecy=cy-wall_true
    if abs(ecx)<=20 and abs(ecy)<=20: ok_right+=1
    print(f"{name:<12}{'?':>5}{int(d.group(1)):>4}{cx:>9.0f}{289:>6}{ecx:>+6.0f}{cy:>7.0f}{wall_true:>6.0f}{ecy:>+6.0f}")
print(f"\n判据 ③（|误差|≤20 场地单位）在合成场景：{ok_right}/{len(conds)} 通过")
print("说明：这是**算法层**的验证（合成图有真值）；真实相机素材因硬件损坏暂时无法测。")
