// ===========================================================================
// geometry_frame.hpp —— **几何帧线协议**（v3 wire format）的解码
//
// 这一份是"PL 说什么"到"决策吃什么"之间的**全部翻译**。协议定义在
// `core/standard/vision/vision_frame.py`（Python 真值）与
// `core/fpga/rtl/vision_pkg.vh`（RTL 生产者）里，这里是**第三个**独立实现 ——
// 三者必须逐字段一致。
//
// 帧布局（`vision_frame_size(n_obs, n_gaps)`）：
//
//     偏移   长度  内容
//     ----   ----  ------------------------------------------------------------
//     0      32    头： magic(<I) version(<H) flags(<H) seq(<I)
//                       n_obs(<I) n_gaps(<I) img_w(<H) img_h(<H) stamp_ns(<Q)
//     32     16    角色： x, y, radius, flags  —— **全部 Q6 定标（×64）**
//     48     24    目标： x, y, radius, half_w, half_h, flags
//     72     36·n_obs   障碍： id, shape, type, x, y, radius, half_w, half_h, rotation
//     ...    36·n_gaps  缺口： 同障碍
//     end-4  4     CRC32（zlib/IEEE，多项式 0xEDB88320）覆盖前面全部字节
//
// ⚠ 所有几何量都是 **Q6**（`VISION_Q = 64`）：解码后必须除以 64 才是世界/像素
//   单位。RTL 侧是右移 6 位（`VISION_Q_DIV`），Python 侧是 `_uq()`。
//   少除这一次，坐标会整体放大 64 倍 —— 而"决策输出一个方向"照样有值，
//   完全不报错，只是全错。
//
// ⚠ 长度与 CRC **必须校验**。协议的设计原则是"任何不一致都抛异常，绝不猜"
//   （Python 侧 `decode_vision_frame` 的 docstring 就是这么写的）。
//   静默接受一帧被截断的数据，比直接崩掉危险得多。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vision_board {

// ---------------------------------------------------------------------------
// 协议常量（必须与 core/standard/vision/vision_frame.py 逐字一致）
// ---------------------------------------------------------------------------
constexpr uint32_t kVisionMagic = 0x31564842u;      // 'BHV1' 小端
constexpr uint16_t kVisionProtocolVersion = 1;
constexpr int      kVisionQ = 64;                   // Q6：×64

constexpr size_t kHeaderBytes = 32;
constexpr size_t kPlayerBytes = 16;
constexpr size_t kTargetBytes = 24;
constexpr size_t kObstacleBytes = 36;
constexpr size_t kGapBytes = 36;
constexpr size_t kTrailerBytes = 4;

//: 角色/目标记录的 flags 位。
enum ActorFlag : uint32_t { kActorValid = 1u << 0, kActorOccluded = 1u << 1 };
//: 缺口记录的 flags 位。
enum GapFlag : uint32_t { kGapReliable = 1u << 0, kGapOccluded = 1u << 1 };
//: 头里的 flags 位。
enum FrameFlag : uint32_t {
    kFrameUnitsM = 1u << 0,      // 有这一位才是**场地单位**；否则是像素
    kFramePlayer = 1u << 1,
    kFrameTarget = 1u << 2,
};

//: 形状编码。`SHAPE_CODES = {"circle": 0, "rect": 1}`。
enum ShapeCode : int32_t { kShapeCircle = 0, kShapeRect = 1 };

// ---------------------------------------------------------------------------
// 帧头
// ---------------------------------------------------------------------------
struct FrameHeader {
    uint32_t magic = 0;
    uint16_t version = 0;
    uint16_t flags = 0;
    uint32_t seq = 0;
    uint32_t n_obs = 0;
    uint32_t n_gaps = 0;
    uint16_t img_w = 0;
    uint16_t img_h = 0;
    uint64_t stamp_ns = 0;

    bool units_are_field() const { return (flags & kFrameUnitsM) != 0; }
    bool has_player() const { return (flags & kFramePlayer) != 0; }
};

//: 从几何帧字节里解头。字节不够或 magic 不对时抛 std::runtime_error。
FrameHeader parse_header(const std::vector<uint8_t>& bytes);

// ---------------------------------------------------------------------------
// 记录
// ---------------------------------------------------------------------------
/// 障碍记录（36 B）。字段顺序见 `_OBSTACLE = struct.Struct("<9i")`。
///
/// ⚠ 障碍记录里**没有** flags 字段，别凭缺口那边的印象臆造一个。
struct GeometryObstacle {
    int32_t id = 0;
    int32_t shape = 0;
    int32_t type_id = 0;
    //: **已经除过 64** 的世界/像素单位（不是原始 Q6 整数）。
    double x = 0.0, y = 0.0;
    double radius = 0.0;
    double half_w = 0.0, half_h = 0.0;
    double rotation = 0.0;

    bool is_rect() const { return shape == kShapeRect; }
    bool is_circle() const { return shape == kShapeCircle; }
};

/// 缺口记录（36 B）—— 与障碍**同字节布局、不同语义**。
/// 名字跟着 Python 侧 `_GAP` 的解包走：`gid, cx, cy, w, axis, ba, bb, bt, gflags`。
/// 决策目前不用缺口（它自己从两段墙反算），但解码要正确，否则将来接上会踩坑。
struct GeometryGap {
    int32_t id = 0;
    double cx = 0.0, cy = 0.0;
    double width = 0.0;
    double axis = 0.0;
    int32_t blocker_a = 0;
    int32_t blocker_b = 0;
    int32_t blocked_by_type = 0;
    bool reliable = false;
    bool occluded = false;
};

struct GeometryPlayer {
    bool present = false;
    double x = 0.0, y = 0.0, radius = 0.0;
    bool occluded = false;
};

struct GeometryFrame {
    FrameHeader header;
    GeometryPlayer player;
    std::vector<GeometryObstacle> obstacles;
    std::vector<GeometryGap> gaps;
};

// ---------------------------------------------------------------------------

/// 整帧字节数。**契约的一部分**，RTL/驱动都按它分配与校验。
size_t geometry_frame_size(uint32_t n_obs, uint32_t n_gaps);

/// zlib/IEEE CRC32（多项式 0xEDB88320，初值 0xFFFFFFFF，末尾取反）。
/// 与 Python 的 ``zlib.crc32`` / RTL 的 CRC 逐位一致。
uint32_t crc32(const uint8_t* data, size_t n);

/// 字节 → :class:`GeometryFrame`。**长度、magic、版本、CRC 四道都校验**，
/// 任何一道不过就抛 std::runtime_error（不截断、不猜测、不"尽力解析"）。
GeometryFrame parse_geometry_frame(const std::vector<uint8_t>& bytes);

}   // namespace vision_board
