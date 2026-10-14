# =============================================================================
# 把 `vision_axi_ip` 打包成块设计能消费的 IP
#
# 这一步是整条"块设计"路的**关键不确定点**：块设计要能认出模块的 AXI-Lite
# 从口，才有接口引脚可连。`vision_axi_ip` 只改了时钟/复位名（→ s_axi_aclk /
# s_axi_aresetn），逻辑一个字节没动。
#
# 产物：$out_dir/ip_vision/ 下的 IP（供 bd_vision.tcl 用）
# =============================================================================
set rtl_dir [lindex $argv 0]
set bd_dir  [lindex $argv 1]
set out_dir [lindex $argv 2]

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
