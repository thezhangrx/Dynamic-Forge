// ===========================================================================
// vision_pkg.vh —— 视觉帧线协议的**硬件侧常量**
//
// 这个文件里的每一个数都必须等于 Python 侧
// `core/standard/vision/vision_frame.py: describe_vision_protocol()` 的值。
// 有一条测试跨语言核对它们（core/fpga/tests/test_rtl_contract.py）：
// **HDL 与 Python 一旦漂移就报错**，而不是等上板才发现字段错位。
//
// 布局（小端，全部以字节为单位；见移植规划 §4）
//   头 32 B: magic(4) version(2) flags(2) seq(4) n_obs(4) n_gaps(4)
//            img_w(2) img_h(2) stamp_ns(8)
//   角色 16 B: x, y, radius, flags            (int32 × 4)
//   目标 24 B: x, y, radius, half_w, half_h, flags   (int32 × 6)
//   障碍 36 B: id, shape, type_id, x, y, radius, half_w, half_h, rotation (int32 × 9)
//   缺口 36 B: id, cx, cy, width, axis, ba, bb, type_code, flags (int32 × 9)
//   尾    4 B: crc32
//   总长 = 76 + 36*n_obs + 36*n_gaps
//
// **没有速度字段**：PL 不知道 `speed`（速度上限，配置量），若编进去，
// PS 侧解出的 PlayerState 会因 speed_measured > speed 而自校验失败。
// 速度由 PS 从相邻帧 stamp 之差推。理由见移植规划 §3 与 vision_frame.py 顶部。
// ===========================================================================

`ifndef VISION_PKG_VH
`define VISION_PKG_VH

// ---- 协议标识 -------------------------------------------------------------
// 'BHV1' 小端 = 0x31564842
`define VISION_MAGIC            32'h31564842
`define VISION_VERSION          16'd1

// 几何定标：wire = round(真实值 * Q)。Q=64 → 量化上限 1/(2*64) = 0.0078125
`define VISION_Q                6'd6          // log2(64)：右移即可
`define VISION_Q_DIV            6'd64

// ---- 记录尺寸（字节）-----------------------------------------------------
`define VISION_HEADER_BYTES     32
`define VISION_PLAYER_BYTES     16
`define VISION_TARGET_BYTES     24
`define VISION_OBSTACLE_BYTES   36
`define VISION_GAP_BYTES        36
`define VISION_TRAILER_BYTES    4
`define VISION_FIXED_BYTES      76      // 32+16+24+4

// ---- 头 flags 位 ----------------------------------------------------------
`define VF_UNITS_M              0       // 1 = 单位是米，0 = 像素
`define VF_PLAYER_VALID         1
`define VF_TARGET_VALID         2

// ---- 角色/目标记录 flags 位 ----------------------------------------------
`define VAF_VALID               0
`define VAF_OCCLUDED            1

// ---- 缺口记录 flags 位 ---------------------------------------------------
`define VGF_RELIABLE            0
`define VGF_OCCLUDED            1

// ---- shape / type 编码（复用 standard.obstacle，只追加不重排）------------
`define VSHAPE_CIRCLE           0
`define VSHAPE_RECT             1
`define VTYPE_NONE              0
`define VTYPE_MOVING_BLOCK      1
`define VTYPE_WALL_WITH_GAP     2
`define VTYPE_SMALL_OBSTACLES   3
`define VTYPE_CORRIDOR          4
`define VTYPE_CROSS_TRAFFIC     5

// ---- 画面参数（按板卡与相机定）------------------------------------------
`define VISION_MAX_W            16'd640
`define VISION_MAX_H            16'd480
`define VISION_W_BITS           10          // 640 < 2^10
`define VISION_H_BITS           9           // 480 < 2^9
// 一帧最多几条记录（BRAM/时延的取舍点，先给够用的上限）
`define VISION_MAX_OBSTACLES    6'd32
`define VISION_MAX_GAPS         6'd16

`endif // VISION_PKG_VH
