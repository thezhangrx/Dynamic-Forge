// ===========================================================================
// ppm_reader.hpp —— 读 P6 PPM（二进制 RGB）
//
// 为什么需要它：真摄像头那条路的第一个环节是"把图弄成像素"。
// 板子上装了 `v4l2-ctl` 与 `ffmpeg`，所以最省事的是
//
//     ffmpeg -f v4l2 -i /dev/video0 -frames:v 1 out.ppm
//
// 然后 `vision_board_devmem --ppm out.ppm` 就能把这张真图跑完整条
// 解码 → 决策 → 键盘。PPM 也是 `core/fpga/ref/export_photos.py` 产出的格式，
// 所以"真照片语料"和"真摄像头帧"走同一条入口。
//
// 只支持 P6 / maxval 255 —— 够用，而且**不允许静默降级**：遇到别的变体就报错，
// 免得把一张灰度图或 16 位图当成 RGB 喂进去。
// ===========================================================================
#ifndef VISION_BOARD_PPM_READER_HPP
#define VISION_BOARD_PPM_READER_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace vision_board {

//: 读 P6 PPM。成功返回 true；失败返回 false 并把原因写进 `why`。
//: `rgb` 会被 resize 到 w*h*3。
bool read_ppm_p6(const std::string& path, int* w, int* h,
                 std::vector<uint8_t>* rgb, std::string* why);

}   // namespace vision_board

#endif   // VISION_BOARD_PPM_READER_HPP
