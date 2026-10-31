# =============================================================================
# 把 `vision_axi_ip` 打包成块设计能消费的 IP
#
# 这一步是整条"块设计"路的**关键不确定点**：块设计要能认出模块的 AXI-Lite
# 从口，才有接口引脚可连。`vision_axi_ip` 只改了时钟/复位名（→ s_axi_aclk /
# s_axi_aresetn），逻辑一个字节没动。
#
# 产物：$out_dir/ip_vision/ 下的 IP（供 bd_vision.tcl 用）
# =============================================================================
#: 路径：命令行 `-tclargs` 给了就用，没给就用下面的默认值
#: （**GUI 的 `Tools → Run Tcl Script…` 没有 argv，也能直接跑**）
#: ⚠ 不能用 `[expr {cond} ? "C:/..." : "D:/..."]`：花括号里的 `expr` 把引号里的
#: 路径当**裸词**解析，报 `invalid bareword "C"`（2026-10-25 实测，pack.log）。
#: 改成"先给默认值，再按 argv 覆盖"。
set rtl_dir "C:/pl_build/rtl"
set bd_dir  "C:/pl_build/bd"
set out_dir "C:/pl_build/out"
if {[info exists argv] && [llength $argv] > 0} { set rtl_dir [lindex $argv 0] }
if {[info exists argv] && [llength $argv] > 1} { set bd_dir  [lindex $argv 1] }
if {[info exists argv] && [llength $argv] > 2} { set out_dir [lindex $argv 2] }
#: 报告文件名用；bd_pack 原来没有 tag，给个固定值
set tag "pack"

set prj $out_dir/bd_prj
file delete -force $prj
create_project -force bd_prj $prj -part xc7z030ffg676-2
set_property target_language Verilog [current_project]
add_files -norecurse [glob $rtl_dir/*.v]
add_files -norecurse [glob $bd_dir/*.v]
set_property include_dirs "$rtl_dir $bd_dir" [current_fileset]

# 先确认包装层能综合（这一步不过，后面都白搭）
synth_design -top vision_axi_ip -part xc7z030ffg676-2 \
    -include_dirs "$rtl_dir $bd_dir"
puts "@@@ 包装层综合通过 @@"
# ---- 存储映射取证（R7：布局报"分布式 RAM 超量"时必须能点名）----------------
#: `[Place 30-640] requires 63447 LUT as Memory` 这种错误只说总数，不说是谁。
#: 这两个报告把**每个**被推断的存储按模块列出来（含 RAMB/LUTRAM 归属），
#: 是"改哪一个 / 加不加 ram_style"的唯一依据。产物在 out/ 下。
report_ram_utilization -file $out_dir/${tag}_ram_pack.rpt
report_utilization -hierarchical -file $out_dir/${tag}_util_pack.rpt
puts "@@@ 存储映射报告已写 ${tag}_ram_pack.rpt / ${tag}_util_pack.rpt @@"
close_design

# ---- 打包成 IP -------------------------------------------------------------
set ipdir $out_dir/ip_vision
file delete -force $ipdir
ipx::package_project -root_dir $ipdir -vendor user -library ip \
    -taxonomy /UserIP -import_files
set core [ipx::current_core]
puts "@@@ 打包完成：$core @@"

puts "@@@ 推断出的总线接口 @@@"
foreach bi [ipx::get_bus_interfaces -of_objects $core] {
    set n [get_property NAME $bi]
    set t {}
    catch {set t [get_property BUS_TYPE_NAME $bi]}
    set ifn {}
    catch {set ifn [get_property BUS_TYPE $bi]}
    puts "    bus=$n  BUS_TYPE_NAME=$t  BUS_TYPE=$ifn"
}
puts "@@@ 时钟/复位接口的关联关系 @@@"
foreach bi [ipx::get_bus_interfaces -of_objects $core] {
    set n [get_property NAME $bi]
    set ab {}
    catch {set ab [get_property ASSOCIATED_BUSIF $bi]}
    puts "    $n : ASSOCIATED_BUSIF=$ab"
}
ipx::create_xgui_files $core
ipx::update_checksums $core
ipx::save_core $core
puts "@@@IP_PACK_OK@@@"
exit
