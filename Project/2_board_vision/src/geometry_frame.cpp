// ===========================================================================
// geometry_frame.cpp —— 几何帧线协议的解码（纯字节，无平台依赖）
//
// 只做三件事：读小端整数、按 Q6 反定标、校验（长度 / magic / 版本 / CRC）。
// 与 `core/standard/vision/vision_frame.py` 的 `decode_vision_frame` 一一对应。
// ===========================================================================
#include "geometry_frame.hpp"

#include <stdexcept>

namespace vision_board {

namespace {

inline uint32_t rd_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}
inline uint16_t rd_u16(const uint8_t* p) {
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0])
         | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint64_t rd_u64(const uint8_t* p) {
    return static_cast<uint64_t>(rd_u32(p))
         | (static_cast<uint64_t>(rd_u32(p + 4)) << 32);
}
inline int32_t rd_i32(const uint8_t* p) {
    return static_cast<int32_t>(rd_u32(p));
}

/// Q6 整数 → 浮点。= Python 的 `_uq()`，= RTL 的右移 6 位。
inline double uq(int32_t v) {
    return static_cast<double>(v) / static_cast<double>(kVisionQ);
}

/// 解一条障碍记录（36 B）。
GeometryObstacle rd_obstacle(const uint8_t* p) {
    GeometryObstacle o;
    o.id = rd_i32(p + 0);
    o.shape = rd_i32(p + 4);
    o.type_id = rd_i32(p + 8);
    o.x = uq(rd_i32(p + 12));
    o.y = uq(rd_i32(p + 16));
    o.radius = uq(rd_i32(p + 20));
    o.half_w = uq(rd_i32(p + 24));
    o.half_h = uq(rd_i32(p + 28));
    o.rotation = uq(rd_i32(p + 32));
    return o;
}

/// 解一条缺口记录（36 B）—— 同布局、不同语义。
/// 对应 Python：`gid, cx, cy, w, axis, ba, bb, bt, gflags = _GAP.unpack_from(...)`
GeometryGap rd_gap(const uint8_t* p) {
    GeometryGap g;
    g.id = rd_i32(p + 0);
    g.cx = uq(rd_i32(p + 4));
    g.cy = uq(rd_i32(p + 8));
    g.width = uq(rd_i32(p + 12));
    g.axis = uq(rd_i32(p + 16));
    g.blocker_a = rd_i32(p + 20);
    g.blocker_b = rd_i32(p + 24);
    g.blocked_by_type = rd_i32(p + 28);
    const uint32_t fl = static_cast<uint32_t>(rd_i32(p + 32));
    g.reliable = (fl & kGapReliable) != 0;
    g.occluded = (fl & kGapOccluded) != 0;
    return g;
}

}  // namespace

size_t geometry_frame_size(uint32_t n_obs, uint32_t n_gaps) {
    return kHeaderBytes + kPlayerBytes + kTargetBytes
         + kObstacleBytes * static_cast<size_t>(n_obs)
         + kGapBytes * static_cast<size_t>(n_gaps)
         + kTrailerBytes;
}

uint32_t crc32(const uint8_t* data, size_t n) {
    // 多项式 0xEDB88320（反射形式）= zlib.crc32。逐位实现，不用查表 ——
    // 板上每帧只调一次、几百字节，省一张 1 KB 表更划算，也少一处不一致的来源。
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) {
        c ^= data[i];
        for (int b = 0; b < 8; ++b) {
            c = (c & 1u) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1);
        }
    }
    return c ^ 0xFFFFFFFFu;
}

FrameHeader parse_header(const std::vector<uint8_t>& bytes) {
    // 头 32 B（见 core/fpga/rtl/vision_pkg.vh；Python 侧是 vision_frame.py 的 HEADER_BYTES）
    if (bytes.size() < kHeaderBytes) {
        throw std::runtime_error("几何帧不足 32 字节，解不出帧头");
    }
    FrameHeader h;
    h.magic    = rd_u32(&bytes[0]);
    h.version  = rd_u16(&bytes[4]);
    h.flags    = rd_u16(&bytes[6]);
    h.seq      = rd_u32(&bytes[8]);
    h.n_obs    = rd_u32(&bytes[12]);
    h.n_gaps   = rd_u32(&bytes[16]);
    h.img_w    = rd_u16(&bytes[20]);
    h.img_h    = rd_u16(&bytes[22]);
    h.stamp_ns = rd_u64(&bytes[24]);
    if (h.magic != kVisionMagic) {
        throw std::runtime_error("几何帧 magic 不对：期望 'BHV1'");
    }
    return h;
}

GeometryFrame parse_geometry_frame(const std::vector<uint8_t>& bytes) {
    GeometryFrame f;
    f.header = parse_header(bytes);          // magic + 最小长度

    // 版本：不许"尽力解析"—— 未知版本的同名字段含义可能已经变了
    if (f.header.version != kVisionProtocolVersion) {
        throw std::runtime_error(
            "视觉帧版本 " + std::to_string(f.header.version)
            + " 不是本实现支持的 " + std::to_string(kVisionProtocolVersion));
    }

    const size_t want = geometry_frame_size(f.header.n_obs, f.header.n_gaps);
    if (bytes.size() != want) {
        throw std::runtime_error(
            "几何帧长度不符：收到 " + std::to_string(bytes.size())
            + " 字节，按 n_obs=" + std::to_string(f.header.n_obs)
            + " n_gaps=" + std::to_string(f.header.n_gaps)
            + " 应为 " + std::to_string(want) + " 字节");
    }

    const uint32_t crc_got = rd_u32(&bytes[want - kTrailerBytes]);
    const uint32_t crc_want = crc32(bytes.data(), want - kTrailerBytes);
    if (crc_got != crc_want) {
        throw std::runtime_error("几何帧 CRC32 不符（帧被截断/篡改）");
    }

    size_t off = kHeaderBytes;

    // 角色：**永远占 16 字节**，有没有内容看头里的 kFramePlayer 位。
    {
        const int32_t px = rd_i32(&bytes[off + 0]);
        const int32_t py = rd_i32(&bytes[off + 4]);
        const int32_t pr = rd_i32(&bytes[off + 8]);
        const int32_t pf = rd_i32(&bytes[off + 12]);
        off += kPlayerBytes;
        if (f.header.has_player()) {
            f.player.present = true;
            f.player.x = uq(px);
            f.player.y = uq(py);
            f.player.radius = uq(pr);
            f.player.occluded = (static_cast<uint32_t>(pf) & kActorOccluded) != 0;
        }
    }

    off += kTargetBytes;                    // 目标：决策用不上，跳过（但仍要占位）

    f.obstacles.reserve(f.header.n_obs);
    for (uint32_t i = 0; i < f.header.n_obs; ++i) {
        f.obstacles.push_back(rd_obstacle(&bytes[off]));
        off += kObstacleBytes;
    }
    f.gaps.reserve(f.header.n_gaps);
    for (uint32_t i = 0; i < f.header.n_gaps; ++i) {
        f.gaps.push_back(rd_gap(&bytes[off]));
        off += kGapBytes;
    }
    return f;
}

}  // namespace vision_board
