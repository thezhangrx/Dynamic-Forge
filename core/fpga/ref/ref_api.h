/* ===========================================================================
 * ref_api.h —— C 参考模型对外的**协同仿真接口**
 *
 * 谁在用：core/fpga/cosim/tb_vision.cpp（Verilator 测试台）。
 * 用法：把 vision_ref.c 用 `-DVR_NO_MAIN` 编译成 .o，和 RTL 模型链进同一个
 * 可执行文件；两边吃同一帧像素，逐位比对。
 *
 * 约束：这里的**尺寸常量必须与 vision_ref.c 内部的一致**，否则
 *      `vr_ref_process()` 复制的数组会越界。有一条测试守着
 *      （core/fpga/tests/test_vision_ref.py::test_ref_api_matches_internal_limits）。
 * =========================================================================== */
#ifndef VR_REF_API_H
#define VR_REF_API_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VR_API_MAX_W         640
#define VR_API_MAX_H         480
#define VR_API_MAX_OBSTACLES 32
#define VR_API_MAX_GAPS      16
#define VR_API_MAX_BANDS     64

typedef struct {
    int width, height;
    int n_bands;
    int band_y0[VR_API_MAX_BANDS], band_y1[VR_API_MAX_BANDS];
    int x_min, x_max;
    int n_obstacles, n_gaps;
    int32_t obstacles[VR_API_MAX_OBSTACLES][9];
    int32_t gaps[VR_API_MAX_GAPS][9];
    int player_valid;
    int32_t player[4];
    int32_t blob_x0, blob_x1, blob_y0, blob_y1, blob_area;
    int n_labels;              /* 亮掩膜的候选连通域数（RTL 标签表的使用量） */
    size_t frame_bytes;
} VrRefResult;

/* rgb：交错 RGB（color=1）或灰度（color=0）。
 * gray_out/wall_out/bright_out：w*h 字节；row_count_out：h 个 uint16；
 * frame_out/frame_cap：几何帧字节缓冲（可为 NULL）。
 * 返回 0 成功，-1 内存不足。 */
int vr_ref_process(const uint8_t *rgb, int w, int h, int color,
                   uint8_t *gray_out, uint8_t *wall_out, uint8_t *bright_out,
                   uint16_t *row_count_out,
                   VrRefResult *res, uint8_t *frame_out, size_t frame_cap);

/* ===========================================================================
 * 运行时可配参数 —— **必须与 rtl/vision_top.v 的配置寄存器地址映射一致**
 *
 * 为什么要有这一层：真机上光照一变，绝对阈值就会假阳性（见 docs/vision/
 * adaptive.md）。目标板上的做法是 PS 通过 AXI-Lite 写这些寄存器，所以
 * C 参考模型也得能接受同一组参数，否则"改了阈值之后 RTL 还对不对"没法验证。
 *
 * 地址映射（rtl/vision_top.v 的 cfg_addr，见 vision_axi.v）：
 *   0..6   wall_rb, wall_min_r, wall_min_g, bright_min, bright_spread,
 *          green_gm, green_min
 *   7..16  band_row_pct, band_join_rows, band_min_rows, field_col_min,
 *          wall_col_pct, wall_col_min, edge_min, edge_min_cols,
 *          min_seg_px, min_gap_px
 *   17..20 player_min_area, player_max_area, min_fill_pct, max_aspect_pct
 *   31     状态（只读）：bit0 busy, bit1 frame_done
 *
 * **`edge_avg` 不在这里**：它决定"持续台阶"的移位寄存器级数（RTL 里是
 * 结构，两级），运行时改不了。契约测试钉住它等于 2。
 *
 * ⚠ 每一项都必须等于 core/vision/wall_with_gap.py 的 `WallWithGapParams` 默认值
 *   —— 那是"语义真值"。早先这里把 wall 的 G 下限写成 110（Python 是 60）、
 *   bright 少了 `hi-lo < bright_spread`、green 写成 50/90（Python 是 10/90），
 *   三处都和真值不一致；有测试钉住（test_rtl_contract.py 的参数表核对）。
 * =========================================================================== */
typedef struct {
    /* ① 逐像素分类（对应 WallWithGapParams 的 ① 段）*/
    int wall_rb;            /* R - B 下限（橙墙）*/
    int wall_min_r;         /* 墙：R 下限 */
    int wall_min_g;         /* 墙：G 下限（比 R 低得多）*/
    int bright_min;         /* 亮：min(R,G,B) 下限 */
    int bright_spread;      /* 亮：max-min 上限 —— 高饱和色不算"白" */
    int green_gm;           /* 绿：G - max(R,B) 下限 */
    int green_min;          /* 绿：G 下限 */
    /* ②..⑥ 检测参数 */
    int band_row_pct, band_join_rows, band_min_rows, field_col_min;
    int wall_col_pct, wall_col_min, edge_min, edge_min_cols;
    int min_seg_px, min_gap_px;
    int player_min_area, player_max_area, player_min_fill_pct, player_max_aspect_pct;
} VrRefParams;

void vr_ref_default_params(VrRefParams *p);

/* seq / stamp_ns 会写进几何帧头。RTL 侧对应 AXI 的 0x0C4 / 0x0C8+0x0CC；
 * 两边给同一个值，帧字节才逐位一致（tb_axi 就是这么验的）。 */
int vr_ref_process_ex(const uint8_t *rgb, int w, int h, int color,
                      uint8_t *gray_out, uint8_t *wall_out, uint8_t *bright_out,
                      uint16_t *row_count_out,
                      VrRefResult *res, uint8_t *frame_out, size_t frame_cap,
                      const VrRefParams *params,
                      uint32_t seq, uint64_t stamp_ns);

#ifdef __cplusplus
}
#endif

#endif /* VR_REF_API_H */
