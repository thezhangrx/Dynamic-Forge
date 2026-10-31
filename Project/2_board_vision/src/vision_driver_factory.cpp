// ===========================================================================
// vision_driver_factory.cpp —— 按 URL 选 :class:`RegIo` 后端
//
//   devmem://0x43C00000   板子上直接 /dev/mem（**不需要 icraft**，默认这条）
//   devmem                省略基址 → 用契约里的 kRegBase
//   axi://zg330aiu?...    icraft 后端（只有定义了 VISION_BOARD_HAVE_ICRAFT 才编进来）
//
// 单独一个编译单元，让 `main.cpp` **不认识任何具体后端** —— 换后端不用动主流程，
// 本机测试也不会被 icraft 拖住。
// ===========================================================================
#include "vision_driver.hpp"

#include <stdexcept>
#include <string>

namespace vision_board {

RegIo* open_reg_io(const std::string& device_url) {
    if (device_url.rfind("devmem", 0) == 0) {
        return devmem_reg_io(device_url);
    }
#ifdef VISION_BOARD_HAVE_ICRAFT
    return icraft_reg_io(device_url);
#else
    throw std::runtime_error(
        "这个二进制没链 icraft 后端，只能开 devmem URL。\n"
        "  板上直接用：--device-url devmem://0x43C00000 （省略基址也可以）");
#endif
}

}   // namespace vision_board
