/* ===========================================================================
 * vision_ref.c —— 视觉链路的 **C 参考模型**（PL 形状：流式、无整帧存灰度）
 *
 * 为什么是 C，不是 Python
 * -----------------------
 * RTL 要逐位比对的对手必须是**能和它一一对应**的实现：
 *   * 整数、定标、位宽都写死；
 *   * 每个函数对应 RTL 里的一块硬件（见下表）；
 *   * 逐帧处理，不整帧存灰度 —— 和硬件一样只能看到"当前像素 + 行缓冲"。
 * Python 那一版（core/vision）留作**语义真值**，用真实录像比对两者：
 * Python 说"缺口中心在这"，C 模型必须给出同一个数（在 §误差预算 内）。
 *
 * 与 RTL 的对应关系
 * -----------------
 *   vision_ref.c                        vision_top.v 里的块
 *   ----------------------------------  ------------------------------
 *   vr_rgb_to_gray()                    灰度化
 *   vr_classify()                       三路分类（wall/bright/green）
 *   vr_col_project()                    列投影（墙的列计数）
 *   vr_row_finish()                     行末处理（行带 → 墙段 → 缺口）
 *   vr_player_feed()                    扫描线连通域 + 一阶/二阶矩
 *   vr_subpixel()                       质心 + 亚像素
 *   vr_frame_begin/end()                帧边界 + 打记录
 *   vr_encode_frame()                   几何帧打包（与 vision_frame.py 同一布局）
 *
 * 生成几何帧的**字节布局与 CRC 规则**必须等于
 * `core/standard/vision/vision_frame.py`（有跨语言测试守着）。
 *
 * ⚠ 状态：**骨架 + 已实现的前端算子**。分类/投影/矩已实现；行带分组、
 *    缺口提取、记录打包沿用 Python 侧语义但**尚未逐帧对账**。
 *    对账脚本见 core/fpga/ref/README.md。
 * ===========================================================================
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "ref_api.h"

/* 对外接口里的尺寸常量与内部必须一致（有测试守着） */
_Static_assert(VR_API_MAX_W == 640 && VR_API_MAX_H == 480, "VR_API 尺寸漂移");
_Static_assert(VR_API_MAX_OBSTACLES == 32 && VR_API_MAX_GAPS == 16, "VR_API 尺寸漂移");
_Static_assert(VR_API_MAX_BANDS == 64, "VR_API 尺寸漂移");

/* ---- 协议常量：必须等于 vision_pkg.vh 与 vision_frame.py ---------------- */
#define VR_MAGIC            0x31564842u      /* 'BHV1' */
#define VR_VERSION          1
#define VR_Q                64
#define VR_HEADER_BYTES     32
#define VR_PLAYER_BYTES     16
#define VR_TARGET_BYTES     24
#define VR_OBSTACLE_BYTES   36
#define VR_GAP_BYTES        36
#define VR_TRAILER_BYTES    4
#define VR_FIXED_BYTES      76

#define VR_MAX_W            640
#define VR_MAX_H            480
#define VR_MAX_OBSTACLES    32
#define VR_MAX_GAPS         16
#define VR_MAX_SEGMENTS     VR_MAX_OBSTACLES      /* 一段墙 = 一条记录 */

/* flags 位号（与 vision_frame.py 一致） */
#define VF_UNITS_M          0
#define VF_PLAYER_VALID     1
#define VF_TARGET_VALID     2
#define VAF_VALID           0
#define VAF_OCCLUDED        1
#define VGF_RELIABLE        0
#define VGF_OCCLUDED        1

#define VSHAPE_CIRCLE       0
#define VSHAPE_RECT         1
#define VTYPE_WALL_WITH_GAP 2

/* ---- 分类阈值（**不做成宏写死**：真机上要按曝光标定，见说明）---------- */
typedef struct {
    int wall_r_minus_b;   /* wall: (R-B) > 阈值 */
    int wall_min_r;       /*       且 R > 阈值 */
    int wall_min_g;       /*       且 G > 阈值（比 R 低得多）*/
    int bright_min;       /* bright: min(R,G,B) > 阈值 */
    int bright_spread;    /*         且 max-min < 阈值（高饱和色不算白）*/
    int green_g_minus;    /* green: (G - max(R,B)) > 阈值 */
    int green_min;
} VrThresholds;

/* **逐项等于** core/vision/wall_with_gap.py 的 `WallWithGapParams` ① 段默认值。
 * 那是语义真值 —— 这里偏一点，RTL 就跟着偏，而"RTL == C 模型"照样全绿。 */
static const VrThresholds VR_DEFAULT_TH = {
    .wall_r_minus_b = 40, .wall_min_r = 110, .wall_min_g = 60,
    .bright_min = 195, .bright_spread = 45,
    .green_g_minus = 10, .green_min = 90,
};

/* ---- 检测参数（**逐条对应** core/vision/wall_with_gap.py 的默认值）------
 * ⚠ 全部是整数：Python 侧写的是 0.05 / 0.25 / 0.45 / 1.6 这些浮点，这里换成
 *   "百分比 / 整数比"，原因是 RTL 里没有浮点，而 `(int)(0.05*w)` 这种写法
 *   在个别宽度上会因为 0.05 不能精确表示而和 `w*5/100` 差一。
 *   对账脚本比的是**几何量**，不是这些系数；系数本身有跨语言测试守着。 */
typedef struct {
    /* 行带 */
    int band_row_pct;       int band_join_rows, band_min_rows;
    /* 场地 x 范围 */
    int field_col_min;
    /* 墙列 */
    int wall_col_pct;       int wall_col_min;
    /* 持续台阶（洗白的墙） */
    int edge_min, edge_avg, edge_min_cols;
    /* 段/缺口 */
    int min_seg_px, min_gap_px;
    /* 角色：fill*100 >= player_min_fill_pct，aspect*100 <= player_max_aspect_pct */
    int player_min_area, player_max_area;
    int player_min_fill_pct, player_max_aspect_pct;
} WallWithGapParamsC;

static const VrRefParams VR_DEFAULT_REF_PARAMS = {
    .wall_rb = 40, .wall_min_r = 110, .wall_min_g = 60,
    .bright_min = 195, .bright_spread = 45,
    .green_gm = 10, .green_min = 90,
    .band_row_pct = 5, .band_join_rows = 4, .band_min_rows = 5,
    .field_col_min = 5,
    .wall_col_pct = 25, .wall_col_min = 2,
    .edge_min = 12, .edge_min_cols = 1,
    .min_seg_px = 8, .min_gap_px = 8,
    .player_min_area = 60, .player_max_area = 1500,
    .player_min_fill_pct = 45, .player_max_aspect_pct = 160,
};

void vr_ref_default_params(VrRefParams *p)
{
    if (p) *p = VR_DEFAULT_REF_PARAMS;
}

static const WallWithGapParamsC VR_DEFAULT_PARAMS = {
    .band_row_pct = 5, .band_join_rows = 4, .band_min_rows = 5,
    .field_col_min = 5,
    .wall_col_pct = 25, .wall_col_min = 2,
    .edge_min = 12, .edge_avg = 2, .edge_min_cols = 1,
    .min_seg_px = 8, .min_gap_px = 8,
    .player_min_area = 60, .player_max_area = 1500,
    .player_min_fill_pct = 45, .player_max_aspect_pct = 160,
};

static inline uint8_t vr_rgb_to_gray(uint8_t r, uint8_t g, uint8_t b)
{
    /* +128 是四舍五入 */
    return (uint8_t)((77 * r + 150 * g + 29 * b + 128) >> 8);
}

/* --------------------------------------------------------------------------
 * 2. 三路分类 —— 对应 RTL 里的比较器组
 *    返回位掩码：(1<<0)=wall  (1<<1)=bright  (1<<2)=green
 * ------------------------------------------------------------------------ */
static inline unsigned vr_classify(uint8_t r, uint8_t g, uint8_t b,
                                   const VrThresholds *th)
{
    unsigned m = 0;
    int mn = (r < g) ? (r < b ? r : b) : (g < b ? g : b);
    int mx = (r > g) ? (r > b ? r : b) : (g > b ? g : b);

    if ((r - b) > th->wall_r_minus_b && r > th->wall_min_r && g > th->wall_min_g)
        m |= 1u << 0;
    /* 高饱和色不算"白"：主场比赛里橙墙的 min 也可能到 196，少了这条
     * `hi-lo` 上限，墙会被同时判成亮色，角色连通域就冒出来。 */
    if (mn > th->bright_min && (mx - mn) < th->bright_spread)
        m |= 1u << 1;
    if ((g - (r > b ? r : b)) > th->green_g_minus && g > th->green_min)
        m |= 1u << 2;
    return m;
}


/* ===========================================================================
 * 视觉链路的算法主体（C 参考模型，第 1 步）：墙/缺口 + 角色连通域
 *
 * 结构说明（与 RTL 的对应）
 * ------------------------
 * 分两遍扫，因为"按行带统计列"本质上需要先知道行带在哪：
 *
 *   第一遍（逐像素，流式）：灰度 → 分类 → 存 1bit 掩膜 + 行直方图
 *   第二遍（按需回读掩膜）：
 *       a) 行带 = 行直方图 → 游程 → 合并 → 丢太薄
 *       b) 场地 x 范围 = 墙带内"墙像素够多"的列
 *       c) 每个行带：列计数 + 持续台阶 → 墙列 → 游程 → 丢太短 → 墙段 → 缺口
 *       d) 角色：亮掩膜上的**游程连通域**（run-based CCL，8 邻接）+ 矩
 *
 * **存整帧是允许的**：实测 PL BRAM 9.3 Mb（≈1190 KB），
 * 灰度 300 KB + 两张 1bit 掩膜 77 KB ≈ 32%。见移植规划 §2.1 的订正说明 ——
 * 早先写"必须流式"是按 1 Mbit 板卡假设的，这块板子不需要那样抠。
 * 真正的流式版（LinkRunCCA 那条路，只存游程不存图）留到 RTL 阶段。
 *
 * 语义**逐条照抄** core/vision/wall_with_gap.py 的 detect_walls / scanline_blobs，
 * 因为第 2 步要拿 Python 当语义真值逐帧对账。
 * ===========================================================================
 */

/* ---- 每帧的缓冲（调用方给，避免动态分配）-------------------------------- */
typedef struct {
    int width, height;

    uint8_t  *gray;        /* width*height */
    uint8_t  *wall;        /* width*height, 0/1 */
    uint8_t  *bright;      /* width*height, 0/1 */

    uint16_t *row_count;   /* height：该行墙像素数 */
    uint16_t *col_count;   /* width：复用于每带的列计数 */

    /* 行带（最多这么多个，够用） */
    int n_bands;
    int band_y0[64], band_y1[64];
    int x_min, x_max;

    /* 输出记录 */
    int n_obstacles, n_gaps;
    int32_t obstacles[VR_MAX_OBSTACLES][9];
    int32_t gaps[VR_MAX_GAPS][9];
    int player_valid;
    int32_t player[4];
    int32_t blob_x0, blob_x1, blob_y0, blob_y1, blob_area;
    int n_labels;          /* 亮掩膜上的候选连通域个数（RTL 的标签表要够大）*/

    uint32_t seq;
    uint64_t stamp_ns;
} VrFrame;


/* --------------------------------------------------------------------------
 * 定标：wire = round(真实值 * Q)，Q = 64。
 *
 * **整条几何链子都是整数运算**，没有 double，没有 libm —— 因为 RTL 里也没有。
 * 这不是"顺手优化"：只要 C 侧还有一步用 double 算完再量化，RTL 就必须在
 * 最后一位上赌它落在哪边；把两边写成**同一串整数运算**，逐位一致才是构造性的，
 * 而不是"测出来没错"。
 *
 * 语义真值仍在 Python 侧（core/vision）；本文件是 RTL 的对手。
 * ------------------------------------------------------------------------ */

/* round(num*scale/den)，den > 0。用 2n+den 的形式保证"恰好半个单位"向上，
 * 与 double 版 vr_q（`v*64 + 0.5` 再截断）在同样的输入上逐位一致。 */
static int32_t vr_scale_round(long num, long den, int scale)
{
    if (den <= 0) return 0;
    long n = num * (long)scale;
    if (n >= 0) return (int32_t)((2 * n + den) / (2 * den));
    return (int32_t)(-((2 * (-n) + den) / (2 * den)));
}

/* 定点 atan2：返回 round(atan2(y_q8, x_q8) * Q)，x_q8 > 0。
 * 只用整数乘除 + 一张 9 点 arctan 表，C 与 RTL 是同一串运算。
 * 线性插值误差 ≤ 0.0013 rad（≈0.3/256），对"墙的倾角"足够；
 * 真实精度参照在 Python 侧，本函数只负责**两边一样**。 */
static int32_t vr_atan2_q6(int32_t y_q8, int32_t x_q8)
{
    /* AT[i] = round(atan(i/8) * 256)，i = 0..8 */
    static const int16_t AT[9] = { 0, 32, 63, 92, 119, 143, 165, 184, 201 };
    if (x_q8 <= 0 || y_q8 == 0) return 0;

    int neg = y_q8 < 0;
    int32_t a = neg ? -y_q8 : y_q8;
    int32_t b = x_q8;
    int inv = 0;
    if (a > b) { int32_t t = a; a = b; b = t; inv = 1; }

    /* r = a/b 的 Q15：a ≤ b ≤ 480*256 → a<<15 < 2^31，32 位放得下 */
    uint32_t r = (uint32_t)(((uint32_t)a << 15) / (uint32_t)b);
    int32_t v;
    if (r >= (1u << 15)) {
        v = AT[8];
    } else {
        int idx = (int)(r >> 12);
        int32_t frac = (int32_t)(r & 4095u);
        v = AT[idx] + (int32_t)(((int32_t)(AT[idx + 1] - AT[idx]) * frac) >> 12);
    }
    if (inv) v = 402 - v;                 /* 402 = round(pi/2 * 256) */
    if (neg) v = -v;

    /* Q8 → Q6，半数远离零 */
    return (v >= 0) ? (v + 2) / 4 : -(((-v) + 2) / 4);
}

/* ---- 小工具：游程 / 丢短 / 合并 ---------------------------------------- */
typedef struct { int a, b; } VrRun;

static int vr_runs(const uint8_t *flags, int n, VrRun *out, int cap)
{
    int k = 0, i = 0;
    while (i < n) {
        if (!flags[i]) { i++; continue; }
        int j = i;
        while (j + 1 < n && flags[j + 1]) j++;
        if (k < cap) { out[k].a = i; out[k].b = j; }
        k++;
        i = j + 1;
    }
    return k;
}

static int vr_drop_short(VrRun *runs, int n, int min_len)
{
    int k = 0;
    for (int i = 0; i < n; i++)
        if (runs[i].b - runs[i].a + 1 >= min_len) runs[k++] = runs[i];
    return k;
}

static int vr_join_close(VrRun *runs, int n, int max_gap)
{
    if (n <= 0) return 0;
    int k = 0;
    for (int i = 1; i < n; i++) {
        if (runs[i].a - runs[k].b - 1 <= max_gap) {
            if (runs[i].b > runs[k].b) runs[k].b = runs[i].b;
        } else {
            runs[++k] = runs[i];
        }
    }
    return k + 1;
}

/* ---- 第一遍：一个像素 -------------------------------------------------- */
static void vr_feed_pixel(VrFrame *f, int x, int y, uint8_t r, uint8_t g, uint8_t b,
                          const VrThresholds *th)
{
    uint8_t gray = vr_rgb_to_gray(r, g, b);
    unsigned cls = vr_classify(r, g, b, th);
    size_t i = (size_t)y * f->width + x;
    f->gray[i]   = gray;
    f->wall[i]   = (uint8_t)((cls & (1u << 0)) ? 1 : 0);
    f->bright[i] = (uint8_t)((cls & (1u << 1)) ? 1 : 0);
    if (f->wall[i]) f->row_count[y]++;
}

/* ---- ② 行带 ------------------------------------------------------------ */
static void vr_find_bands(VrFrame *f, const WallWithGapParamsC *p)
{
    uint8_t flags[VR_MAX_H];
    VrRun runs[VR_MAX_H];
    int limit = (int)(p->band_row_pct * f->width) / 100;
    if (limit < 1) limit = 1;
    for (int y = 0; y < f->height; y++) flags[y] = (uint8_t)(f->row_count[y] >= limit);
    int n = vr_runs(flags, f->height, runs, VR_MAX_H);
    n = vr_join_close(runs, n, p->band_join_rows);
    f->n_bands = 0;
    for (int i = 0; i < n && f->n_bands < 64; i++)
        if (runs[i].b - runs[i].a + 1 >= p->band_min_rows) {
            f->band_y0[f->n_bands] = runs[i].a;
            f->band_y1[f->n_bands] = runs[i].b;
            f->n_bands++;
        }
}

/* ---- ③ 场地 x 范围 ----------------------------------------------------- */
static void vr_find_span(VrFrame *f, const WallWithGapParamsC *p)
{
    f->x_min = 0;
    f->x_max = f->width - 1;
    if (f->n_bands == 0) return;
    int col[VR_MAX_W];
    for (int x = 0; x < f->width; x++) col[x] = 0;
    for (int b = 0; b < f->n_bands; b++)
        for (int y = f->band_y0[b]; y <= f->band_y1[b]; y++)
            for (int x = 0; x < f->width; x++)
                col[x] += f->wall[(size_t)y * f->width + x];
    int lo = -1, hi = -1;
    for (int x = 0; x < f->width; x++)
        if (col[x] >= p->field_col_min) { if (lo < 0) lo = x; hi = x; }
    if (lo >= 0) { f->x_min = lo; f->x_max = hi; }
}

/* ---- 持续台阶：一列在带内相邻 k 行的最大亮度跳变 ---------------------- */
static int vr_col_step_max(const VrFrame *f, int x, int y0, int y1, int k)
{
    int best = 0;
    for (int y = y0; y + k <= y1; y++) {
        int d = (int)f->gray[(size_t)(y + k) * f->width + x]
              - (int)f->gray[(size_t)y * f->width + x];
        if (d < 0) d = -d;
        if (d > best) best = d;
    }
    return best;
}

/* ---- 一段墙的 (cy_q6, thickness, angle_q6) ------------------------------
 * 全整数：cy = Σy/total 直接量化到 Q6；倾角 = 左右两端重心 y 之差 / 间距，
 * 走定点 atan2。这样 C 与 RTL 是同一串运算，逐位一致不靠容差。 */
static void vr_segment_stats(const VrFrame *f, int y0, int y1, int x0, int x1,
                             int32_t *cy_q, int *thick, int32_t *ang_q)
{
    long total = 0, sum_y = 0;
    int tk = 0;
    for (int x = x0; x <= x1; x++) {
        int col = 0;
        for (int y = y0; y <= y1; y++)
            if (f->wall[(size_t)y * f->width + x]) { col++; total++; sum_y += y; }
        if (col > tk) tk = col;
    }
    *cy_q  = total ? vr_scale_round(sum_y, total, VR_Q)
                   : (int32_t)((y0 + y1) * (VR_Q / 2));
    *thick = tk;

    int span = (x1 - x0 + 1) / 4;
    if (span > 20) span = 20;
    if (span < 1) span = 1;
    long ls = 0, rs = 0, ln = 0, rn = 0;
    for (int x = x0; x <= x0 + span - 1 && x <= x1; x++)
        for (int y = y0; y <= y1; y++)
            if (f->wall[(size_t)y * f->width + x]) { ls += y; ln++; }
    for (int x = (x1 - span + 1 > x0 ? x1 - span + 1 : x0); x <= x1; x++)
        for (int y = y0; y <= y1; y++)
            if (f->wall[(size_t)y * f->width + x]) { rs += y; rn++; }
    /* 左右重心用 Q8 表示：Σy ≤ 480*20*480 ≈ 4.6e6，<<8 后仍在 32 位内 */
    int32_t lq = ln ? vr_scale_round(ls, ln, 256) : *cy_q * 4;
    int32_t rq = rn ? vr_scale_round(rs, rn, 256) : *cy_q * 4;
    int dx = (x1 - span + 1) - (x0 + span - 1);
    *ang_q = (dx > 0) ? vr_atan2_q6((int32_t)(rq - lq), (int32_t)(dx << 8)) : 0;
    if (getenv("VR_DEBUG_SEG"))
        fprintf(stderr, "C  seg x=[%d,%d] span=%d ls=%ld ln=%ld rs=%ld rn=%ld lq=%d rq=%d dx=%d ang=%d\n",
                x0, x1, span, ls, ln, rs, rn, lq, rq, dx, *ang_q);
}

/* ---- ④ 墙段 + 缺口 ----------------------------------------------------- */
static void vr_extract_walls(VrFrame *f, const WallWithGapParamsC *p)
{
    for (int b = 0; b < f->n_bands; b++) {
        int y0 = f->band_y0[b], y1 = f->band_y1[b];
        int h = y1 - y0 + 1;

        for (int x = 0; x < f->width; x++) f->col_count[x] = 0;
        for (int x = f->x_min; x <= f->x_max; x++)
            for (int y = y0; y <= y1; y++)
                f->col_count[x] += f->wall[(size_t)y * f->width + x];

        int limit = (int)(p->wall_col_pct * h) / 100;
        if (limit < p->wall_col_min) limit = p->wall_col_min;
        int edge_limit = p->edge_min * (p->edge_avg > 1 ? p->edge_avg : 1);

        static uint8_t flags[VR_MAX_W];
        int nw = f->x_max - f->x_min + 1;
        for (int i = 0; i < nw; i++) {
            int x = f->x_min + i;
            int c = f->col_count[x];
            int e = (c >= p->edge_min_cols) ? vr_col_step_max(f, x, y0, y1, p->edge_avg) : 0;
            flags[i] = (uint8_t)((c >= limit) || (e >= edge_limit));
        }
        VrRun runs[VR_MAX_W];
        int n = vr_runs(flags, nw, runs, VR_MAX_W);
        n = vr_drop_short(runs, n, p->min_seg_px);
        if (n == 0) continue;

        /* 本带的墙段转成障碍记录，并记下 id 供缺口引用 */
        int seg_count = 0;
        int sid[64]; int32_t scy_q[64];
        for (int i = 0; i < n && f->n_obstacles < VR_MAX_OBSTACLES; i++) {
            int sx0 = runs[i].a + f->x_min, sx1 = runs[i].b + f->x_min;
            int32_t cy_q, ang_q; int thick;
            vr_segment_stats(f, y0, y1, sx0, sx1, &cy_q, &thick, &ang_q);
            int id = f->n_obstacles + 1;          /* 1 起，与 Python 侧的 id 习惯一致 */
            int32_t *o = f->obstacles[f->n_obstacles];
            o[0] = id;
            o[1] = VSHAPE_RECT;
            o[2] = VTYPE_WALL_WITH_GAP;
            /* 字段顺序**必须**与 vision_frame.py 的 _OBSTACLE 一致：
             *   id, shape, type_id, x, y, radius, half_w, half_h, rotation
             * 矩形没有 radius（写 0），位置写在 half_w/half_h。
             * 早先把 half_w 放在 index 5 —— 整体错位一格，被
             * decode_vision_frame 的 validate() 当场拒掉（half_h 解成负数）。 */
            o[3] = (sx0 + sx1) * (VR_Q / 2);                     /* x = 段中点 */
            o[4] = cy_q;
            o[5] = 0;                                            /* radius: 矩形没有 */
            o[6] = (sx1 - sx0 + 1) * (VR_Q / 2);                 /* half_w */
            o[7] = (thick > 0 ? thick : h) * (VR_Q / 2);         /* half_h = 厚/2 */
            o[8] = ang_q;
            if (seg_count < 64) { sid[seg_count] = id; scy_q[seg_count] = cy_q; }
            seg_count++;
            f->n_obstacles++;
        }

        /* 缺口：相邻两段之间最宽的那个，且 >= min_gap_px */
        int best = 0, bi = -1;
        for (int i = 0; i + 1 < seg_count; i++) {
            int w = (runs[i + 1].a + f->x_min) - (runs[i].b + f->x_min) - 1;
            if (w >= p->min_gap_px && w > best) { best = w; bi = i; }
        }
        if (bi >= 0 && f->n_gaps < VR_MAX_GAPS) {
            int32_t *gp = f->gaps[f->n_gaps];
            gp[0] = f->n_gaps + 1;
            gp[1] = (runs[bi].b + runs[bi + 1].a + 2 * f->x_min) * (VR_Q / 2); /* cx */
            gp[2] = (scy_q[bi] + scy_q[bi + 1] + 1) >> 1;  /* 两个 Q6 的均值 */
            gp[3] = best * VR_Q;
            gp[4] = 0;
            gp[5] = sid[bi];
            gp[6] = sid[bi + 1];
            gp[7] = VTYPE_WALL_WITH_GAP;
            gp[8] = (1 << VGF_RELIABLE);
            f->n_gaps++;
        }
    }
}

/* ---- ⑤ 角色：游程连通域（8 邻接）+ 矩 -------------------------------- */
/* 一行最多 W/2 = 320 条游程，2048 够用且永不触顶（触顶语义在 RTL 里也要一致，
 * 所以这个上限本身就是契约的一部分）。 */
#define VR_MAX_RUNS 2048
/* 标签表：RTL 侧是一块 256 深的表。实测真实录像 320 帧上最多只出现 3 个亮
 * 连通域，余量极大；但**上限值仍然要一致** —— 触顶后 C 的语义是"这条游程不累积"，
 * RTL 必须照抄。cosim 会比对每帧的标签数。 */
#define VR_MAX_LABELS 256

typedef struct {
    int x0, x1, y;          /* 游程 */
    int label;
} VrCRun;

typedef struct {
    int n, x0, x1, y0, y1;
    long sx, sy;
} VrComp;

/* comp 提到文件作用域：并查集在 union 的那一刻就要把**被并入那一边的历史矩**
 * 搬过来（见 uf_union），否则前几行累加的矩会留在已经不看的节点里。 */
static VrComp vr_comp[VR_MAX_LABELS];
static int uf_parent[VR_MAX_LABELS];

static int uf_find(int a)
{
    while (uf_parent[a] != a) { uf_parent[a] = uf_parent[uf_parent[a]]; a = uf_parent[a]; }
    return a;
}
static void uf_union(int a, int b)
{
    a = uf_find(a); b = uf_find(b);
    if (a == b) return;
    uf_parent[b] = a;
    /* 合并矩与包围盒。原实现只改 parent 不搬矩：被并入的分量在**之前各行**
     * 累出来的 n/sx/sy/bbox 就永远留在 comp[b] 里没人读了 —— 连通域一旦
     * 跨行"先分后合"（斜接、V 形），面积与质心都会偏小。 */
    vr_comp[a].n  += vr_comp[b].n;
    vr_comp[a].sx += vr_comp[b].sx;
    vr_comp[a].sy += vr_comp[b].sy;
    if (vr_comp[b].x0 < vr_comp[a].x0) vr_comp[a].x0 = vr_comp[b].x0;
    if (vr_comp[b].x1 > vr_comp[a].x1) vr_comp[a].x1 = vr_comp[b].x1;
    if (vr_comp[b].y0 < vr_comp[a].y0) vr_comp[a].y0 = vr_comp[b].y0;
    if (vr_comp[b].y1 > vr_comp[a].y1) vr_comp[a].y1 = vr_comp[b].y1;
}

static void vr_extract_player(VrFrame *f, const WallWithGapParamsC *p)
{
    static VrCRun prev[VR_MAX_RUNS], cur[VR_MAX_RUNS];
    VrComp *comp = vr_comp;
    int n_prev = 0, n_label = 0;
    for (int i = 0; i < VR_MAX_LABELS; i++) uf_parent[i] = i;

    for (int y = 0; y < f->height; y++) {
        int n_cur = 0;
        const uint8_t *row = f->bright + (size_t)y * f->width;
        int x = 0;
        while (x < f->width && n_cur < VR_MAX_RUNS) {
            if (!row[x]) { x++; continue; }
            int a = x;
            while (x + 1 < f->width && row[x + 1]) x++;
            cur[n_cur].x0 = a; cur[n_cur].x1 = x; cur[n_cur].y = y;
            cur[n_cur].label = -1;
            n_cur++;
            x++;
        }
        /* 与上一行重叠（8 邻接：允许斜接，即 x 范围各放宽 1）的游程合并 */
        for (int i = 0; i < n_cur; i++) {
            for (int j = 0; j < n_prev; j++) {
                if (cur[i].x0 <= prev[j].x1 + 1 && cur[i].x1 >= prev[j].x0 - 1) {
                    if (cur[i].label < 0) cur[i].label = uf_find(prev[j].label);
                    else uf_union(cur[i].label, prev[j].label);
                }
            }
            if (cur[i].label < 0) {
                if (n_label >= VR_MAX_LABELS) continue;
                comp[n_label].n = 0;
                comp[n_label].x0 = cur[i].x0; comp[n_label].x1 = cur[i].x1;
                comp[n_label].y0 = y; comp[n_label].y1 = y;
                comp[n_label].sx = 0; comp[n_label].sy = 0;
                cur[i].label = n_label++;
            }
        }
        /* 把本行游程累进它所属的分量（用 find 后的根，避免合并后写错人） */
        for (int i = 0; i < n_cur; i++) {
            if (cur[i].label < 0) continue;          /* 标签表满，本行这条丢了 */
            int L = uf_find(cur[i].label);
            VrComp *c = &comp[L];
            int len = cur[i].x1 - cur[i].x0 + 1;
            c->n += len;
            c->sx += (long)(cur[i].x0 + cur[i].x1) * len / 2;
            c->sy += (long)y * len;
            if (cur[i].x0 < c->x0) c->x0 = cur[i].x0;
            if (cur[i].x1 > c->x1) c->x1 = cur[i].x1;
            if (y > c->y1) c->y1 = y;
        }
        memcpy(prev, cur, sizeof(VrCRun) * (size_t)n_cur);
        n_prev = n_cur;
    }

    f->n_labels = n_label;
    /* 选角色：与 Python detect_player 同一套判据与打分（fill × area）。
     * 打分用**交叉相乘比较**（n²/(w·h) > n'²/(w'·h') ⟺ n²·w'·h' > n'²·w·h），
     * 不落浮点 —— double 里 fill*area 的最后一位会影响同分时选谁。 */
    int best = -1;
    long long best_num = 0, best_den = 1;
    for (int i = 0; i < n_label; i++) {
        if (uf_find(i) != i) continue;                 /* 只看根 */
        const VrComp *c = &comp[i];
        if (c->n < p->player_min_area || c->n > p->player_max_area) continue;
        int w = c->x1 - c->x0 + 1, h = c->y1 - c->y0 + 1;
        if (w <= 0 || h <= 0) continue;
        if (c->x0 <= 0 || c->y0 <= 0 || c->x1 >= f->width - 1 || c->y1 >= f->height - 1)
            continue;                                   /* 贴边的大片高光不要 */
        long wh = (long)w * (long)h;
        if (100L * c->n < (long)p->player_min_fill_pct * wh) continue;     /* fill 太低 */
        int mx = (w > h) ? w : h, mn = (w > h) ? h : w;
        if (100L * mx > (long)p->player_max_aspect_pct * mn) continue;     /* 太扁长 */
        long long num = (long long)c->n * (long long)c->n;
        if (num * best_den > best_num * (long long)wh) { best_num = num; best_den = wh; best = i; }
    }
    if (best < 0) return;
    const VrComp *c = &comp[best];
    int w = c->x1 - c->x0 + 1, h = c->y1 - c->y0 + 1;
    f->player[0] = vr_scale_round(c->sx, c->n, VR_Q);
    f->player[1] = vr_scale_round(c->sy, c->n, VR_Q);
    f->player[2] = (int32_t)((w + h) * (VR_Q / 4));   /* 口径 (w+h)/4 */
    f->player[3] = (1 << VAF_VALID);
    f->player_valid = 1;
    f->blob_x0 = c->x0; f->blob_x1 = c->x1;
    f->blob_y0 = c->y0; f->blob_y1 = c->y1; f->blob_area = c->n;
}


/* ---- 帧边界：调用方给缓冲，避免动态分配（RTL 里就是那几块 BRAM）-------- */
static void vr_frame_begin(VrFrame *f, int w, int h,
                           uint8_t *gray, uint8_t *wall, uint8_t *bright,
                           uint16_t *row_count, uint16_t *col_count,
                           uint32_t seq, uint64_t stamp_ns)
{
    memset(f, 0, sizeof(*f));
    f->width = (w > VR_MAX_W) ? VR_MAX_W : w;
    f->height = (h > VR_MAX_H) ? VR_MAX_H : h;
    f->gray = gray; f->wall = wall; f->bright = bright;
    f->row_count = row_count; f->col_count = col_count;
    f->seq = seq; f->stamp_ns = stamp_ns;
    for (int y = 0; y < f->height; y++) row_count[y] = 0;
}

/* ---- 跑完整条链（两遍）------------------------------------------------ */
static void vr_process(VrFrame *f, const WallWithGapParamsC *p)
{
    vr_find_bands(f, p);
    vr_find_span(f, p);
    vr_extract_walls(f, p);
    vr_extract_player(f, p);
}

static uint32_t vr_crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put_u32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static void put_u64(uint8_t *p, uint64_t v)
{ for (int i=0;i<8;i++) p[i]=(uint8_t)(v>>(8*i)); }
static void put_i32(uint8_t *p, int32_t v) { put_u32(p, (uint32_t)v); }

/* 返回写入的字节数；buf 至少要有 vr_frame_size() 字节 */
static size_t vr_encode_frame(const VrFrame *f, uint8_t *buf)
{
    uint16_t flags = 0;
    if (f->player_valid) flags |= (uint16_t)(1u << VF_PLAYER_VALID);

    size_t off = 0;
    put_u32(buf + off, VR_MAGIC);           off += 4;
    put_u16(buf + off, VR_VERSION);         off += 2;
    put_u16(buf + off, flags);              off += 2;
    put_u32(buf + off, f->seq);             off += 4;
    put_u32(buf + off, (uint32_t)f->n_obstacles); off += 4;
    put_u32(buf + off, (uint32_t)f->n_gaps);      off += 4;
    put_u16(buf + off, (uint16_t)f->width);  off += 2;
    put_u16(buf + off, (uint16_t)f->height); off += 2;
    put_u64(buf + off, f->stamp_ns);         off += 8;
    /* 到此 32 B */

    for (int i = 0; i < 4; i++) { put_i32(buf + off, f->player[i]); off += 4; }  /* 16 B */
    for (int i = 0; i < 6; i++) { put_i32(buf + off, 0); off += 4; }             /* target 24 B，本模型不产出 */

    for (int r = 0; r < f->n_obstacles; r++)
        for (int c = 0; c < 9; c++) { put_i32(buf + off, f->obstacles[r][c]); off += 4; }
    for (int r = 0; r < f->n_gaps; r++)
        for (int c = 0; c < 9; c++) { put_i32(buf + off, f->gaps[r][c]); off += 4; }

    uint32_t crc = vr_crc32(buf, off);
    put_u32(buf + off, crc); off += 4;
    return off;
}

static size_t vr_frame_size(int n_obs, int n_gaps)
{
    return (size_t)VR_FIXED_BYTES + VR_OBSTACLE_BYTES * (size_t)n_obs
                                 + VR_GAP_BYTES * (size_t)n_gaps;
}


/* ---- P5(灰度) / P6(彩色) 读取（只给命令行用，cosim 不需要）------------- */
#ifndef VR_NO_MAIN
static uint8_t *load_image(const char *path, int *w, int *h, int *color)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) { fprintf(stderr, "打不开 %s\n", path); return NULL; }
    char magic[3] = {0};
    if (fscanf(fp, "%2s", magic) != 1 || (strcmp(magic, "P5") && strcmp(magic, "P6"))) {
        fprintf(stderr, "%s 不是 P5/P6\n", path); fclose(fp); return NULL;
    }
    *color = (magic[1] == '6');
    int c, vals[2] = {0,0}, n = 0, maxv = 0, seen = 0;
    while (n < 2 && (c = fgetc(fp)) != EOF) {
        if (c == '#') { while ((c = fgetc(fp)) != EOF && c != '\n') {} continue; }
        if (c >= '0' && c <= '9') vals[n] = vals[n]*10 + (c - '0');
        else if (vals[n] > 0) n++;
    }
    while ((c = fgetc(fp)) != EOF) {
        if (c >= '0' && c <= '9') maxv = maxv*10 + (c - '0');
        else if (maxv > 0) { seen = 1; break; }
    }
    if (!seen || maxv <= 0) { fclose(fp); return NULL; }
    *w = vals[0]; *h = vals[1];
    size_t nch = *color ? 3u : 1u;
    size_t want = (size_t)(*w) * (size_t)(*h) * nch;
    uint8_t *img = (uint8_t *)malloc(want);
    if (!img || fread(img, 1, want, fp) != want) {
        fprintf(stderr, "%s 像素数据不足\n", path); free(img); fclose(fp); return NULL;
    }
    fclose(fp);
    return img;
}
#endif /* VR_NO_MAIN */

/* --------------------------------------------------------------------------
 * 命令行覆盖检测参数：`--set NAME=VALUE`，可重复。
 *
 * 为什么需要：Python 侧的 `WallWithGapParams.rescaled(w, h)` 会按分辨率缩放
 * **长度/面积**类参数（640x360 时 s = min(640/640, 360/480) = 0.75：
 * min_seg_px 8→6、band_min_rows 5→4、field_col_min 5→4、player_min_area 60→34 …）。
 * C 模型与 RTL 的默认值是不缩放的 640x480 值，所以对账必须把缩放后的参数
 * 喂进来 —— 否则差的不是算法，是参数。
 * 这也是上板时 PS 必须做的事：算好缩放参数，写进配置寄存器。
 * ------------------------------------------------------------------------ */
typedef struct { const char *name; size_t off; } VrParamSlot;

static const VrParamSlot VR_PARAM_SLOTS[] = {
    { "wall_rb",        offsetof(VrRefParams, wall_rb) },
    { "wall_min_r",     offsetof(VrRefParams, wall_min_r) },
    { "wall_min_g",     offsetof(VrRefParams, wall_min_g) },
    { "bright_min",     offsetof(VrRefParams, bright_min) },
    { "bright_spread",  offsetof(VrRefParams, bright_spread) },
    { "green_gm",       offsetof(VrRefParams, green_gm) },
    { "green_min",      offsetof(VrRefParams, green_min) },
    { "band_row_pct",   offsetof(VrRefParams, band_row_pct) },
    { "band_join_rows", offsetof(VrRefParams, band_join_rows) },
    { "band_min_rows",  offsetof(VrRefParams, band_min_rows) },
    { "field_col_min",  offsetof(VrRefParams, field_col_min) },
    { "wall_col_pct",   offsetof(VrRefParams, wall_col_pct) },
    { "wall_col_min",   offsetof(VrRefParams, wall_col_min) },
    { "edge_min",       offsetof(VrRefParams, edge_min) },
    { "edge_min_cols",  offsetof(VrRefParams, edge_min_cols) },
    { "min_seg_px",     offsetof(VrRefParams, min_seg_px) },
    { "min_gap_px",     offsetof(VrRefParams, min_gap_px) },
    { "player_min_area",           offsetof(VrRefParams, player_min_area) },
    { "player_max_area",           offsetof(VrRefParams, player_max_area) },
    { "player_min_fill_pct",       offsetof(VrRefParams, player_min_fill_pct) },
    { "player_max_aspect_pct",     offsetof(VrRefParams, player_max_aspect_pct) },
    { NULL, 0 }
};

static void apply_param_overrides(VrRefParams *p, int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--set") != 0) continue;
        if (i + 1 >= argc) { fprintf(stderr, "--set 后面要跟 NAME=VALUE\n"); exit(2); }
        const char *kv = argv[++i];
        const char *eq = strchr(kv, '=');
        if (!eq) { fprintf(stderr, "--set 要用 NAME=VALUE 形式：%s\n", kv); exit(2); }
        size_t nlen = (size_t)(eq - kv);
        int hit = 0;
        for (const VrParamSlot *sl = VR_PARAM_SLOTS; sl->name; sl++) {
            if (strlen(sl->name) != nlen || strncmp(sl->name, kv, nlen) != 0) continue;
            *(int *)((char *)p + sl->off) = (int)strtol(eq + 1, NULL, 10);
            hit = 1;
            break;
        }
        if (!hit) { fprintf(stderr, "不认识的参数：%.*s\n", (int)nlen, kv); exit(2); }
    }
}

/* --------------------------------------------------------------------------
 * main：把一帧图像跑完整条链，输出几何帧
 *   用法: vision_ref input.pgm|input.ppm [out.bin] [--params-dump]
 *   支持 P5(灰度) 与 P6(彩色)。**灰度帧检不出墙** —— wall 判据是 R-B>40，
 *   灰度下恒为 0。要验墙的列投影与缺口，激励必须是彩色的（见 ref/README.md）。
 * ------------------------------------------------------------------------ */
#ifndef VR_NO_MAIN
int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "用法: %s image.pgm|ppm [out.bin] [--set NAME=VALUE ...]\n", argv[0]);
        return 2;
    }
    int w = 0, h = 0, color = 0;
    uint8_t *pix = load_image(argv[1], &w, &h, &color);
    if (!pix) return 1;
    if (w > VR_MAX_W || h > VR_MAX_H) {
        fprintf(stderr, "帧 %dx%d 超过上限 %dx%d\n", w, h, VR_MAX_W, VR_MAX_H);
        free(pix); return 1;
    }

    VrRefParams params;
    vr_ref_default_params(&params);
    apply_param_overrides(&params, argc, argv);

    /* 走协同仿真用的同一条路（vr_ref_process_ex）——不再在 main 里重写一遍
     * 流水。这样命令行跑出来的数与 tb 里跑的数是**同一条代码路径**，
     * 不会出现"命令行对、仿真里不对"。 */
    size_t cap = VR_FIXED_BYTES + VR_OBSTACLE_BYTES * (size_t)VR_MAX_OBSTACLES
                               + VR_GAP_BYTES * (size_t)VR_MAX_GAPS;
    uint8_t *buf = (uint8_t *)malloc(cap);
    VrRefResult res;
    if (!buf) { free(pix); return 1; }
    if (vr_ref_process_ex(pix, w, h, color, NULL, NULL, NULL, NULL,
                          &res, buf, cap, &params, 0, 0) != 0) {
        fprintf(stderr, "内存不足\n"); free(buf); free(pix); return 1;
    }

    FILE *out = stdout;
    if (argc >= 3 && argv[2][0] != '-') {
        out = fopen(argv[2], "wb");
        if (!out) { perror(argv[2]); free(buf); free(pix); return 1; }
    }
    fwrite(buf, 1, res.frame_bytes, out);
    if (out != stdout) fclose(out);

    fprintf(stderr, "%s %dx%d(%s): n_obs=%d n_gaps=%d player=%d bands=%d span=[%d,%d] labels=%d\n",
            argv[1], w, h, color ? "color" : "gray",
            res.n_obstacles, res.n_gaps, res.player_valid, res.n_bands,
            res.x_min, res.x_max, res.n_labels);
    free(buf); free(pix);
    return 0;
}
#endif /* VR_NO_MAIN */


/* ===========================================================================
 * 协同仿真 API —— 给 Verilator 测试台（core/fpga/cosim/tb_vision.cpp）用
 *
 * 用 -DVR_NO_MAIN 编译本文件，把它和 Verilator 生成的 RTL 模型链进同一个
 * 可执行文件，两边吃**同一帧同一批像素**，逐位比对这些量：
 *   gray / wall / bright 掩膜、行直方图、行带、场地范围、墙段、缺口、角色、
 *   以及最终几何帧的**全部字节**（含 CRC）。
 * 之所以做成"同一个进程里比"而不是"两边各写文件再 diff"：中间量不必发明
 * 一套 dump 格式，出问题时可以直接打印 RTL 出错的那一拍。
 * =========================================================================== */

/* VrRefResult 的字段定义在 ref_api.h（RTL 测试台也 include 它）。 */

/* 把 AXI 侧那套参数搬到内部的阈值/检测参数结构体上。
 * `edge_avg` 不在这里：它是 RTL 的结构参数（两级移位寄存器），运行时不改。 */
static void vr_params_apply(const VrRefParams *p, VrThresholds *th, WallWithGapParamsC *wp)
{
    if (!p) { *th = VR_DEFAULT_TH; *wp = VR_DEFAULT_PARAMS; return; }
    th->wall_r_minus_b = p->wall_rb;
    th->wall_min_r     = p->wall_min_r;
    th->wall_min_g     = p->wall_min_g;
    th->bright_min     = p->bright_min;
    th->bright_spread  = p->bright_spread;
    th->green_g_minus  = p->green_gm;
    th->green_min      = p->green_min;
    *wp = VR_DEFAULT_PARAMS;                  /* edge_avg 等结构参数取默认 */
    wp->band_row_pct   = p->band_row_pct;
    wp->band_join_rows = p->band_join_rows;
    wp->band_min_rows  = p->band_min_rows;
    wp->field_col_min  = p->field_col_min;
    wp->wall_col_pct   = p->wall_col_pct;
    wp->wall_col_min   = p->wall_col_min;
    wp->edge_min       = p->edge_min;
    wp->edge_min_cols  = p->edge_min_cols;
    wp->min_seg_px     = p->min_seg_px;
    wp->min_gap_px     = p->min_gap_px;
    wp->player_min_area       = p->player_min_area;
    wp->player_max_area       = p->player_max_area;
    wp->player_min_fill_pct   = p->player_min_fill_pct;
    wp->player_max_aspect_pct = p->player_max_aspect_pct;
}

/* 全部输出缓冲由调用方给。rgb 是交错 RGB（color=1）或灰度（color=0）。
 * 返回 0 成功，-1 内存不足。 */
int vr_ref_process_ex(const uint8_t *rgb, int w, int h, int color,
                      uint8_t *gray_out, uint8_t *wall_out, uint8_t *bright_out,
                      uint16_t *row_count_out,
                      VrRefResult *res, uint8_t *frame_out, size_t frame_cap,
                      const VrRefParams *params,
                      uint32_t seq, uint64_t stamp_ns)
{
    VrThresholds th;
    WallWithGapParamsC wp;
    vr_params_apply(params, &th, &wp);

    if (w > VR_MAX_W) w = VR_MAX_W;
    if (h > VR_MAX_H) h = VR_MAX_H;
    size_t npix = (size_t)w * (size_t)h;
    uint8_t  *gray   = (uint8_t *)malloc(npix);
    uint8_t  *wall   = (uint8_t *)malloc(npix);
    uint8_t  *bright = (uint8_t *)malloc(npix);
    uint16_t *rowc   = (uint16_t *)calloc((size_t)h, sizeof(uint16_t));
    uint16_t *colc   = (uint16_t *)calloc((size_t)w, sizeof(uint16_t));
    if (!gray || !wall || !bright || !rowc || !colc) {
        free(gray); free(wall); free(bright); free(rowc); free(colc); return -1;
    }

    VrFrame f;
    vr_frame_begin(&f, w, h, gray, wall, bright, rowc, colc, seq, stamp_ns);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            uint8_t r, g, b;
            if (color) { r = rgb[i*3+0]; g = rgb[i*3+1]; b = rgb[i*3+2]; }
            else       { r = g = b = rgb[i]; }
            vr_feed_pixel(&f, x, y, r, g, b, &th);
        }
    }
    vr_process(&f, &wp);

    if (gray_out)   memcpy(gray_out, gray, npix);
    if (wall_out)   memcpy(wall_out, wall, npix);
    if (bright_out) memcpy(bright_out, bright, npix);
    if (row_count_out) memcpy(row_count_out, rowc, (size_t)h * sizeof(uint16_t));

    memset(res, 0, sizeof(*res));
    res->width = f.width; res->height = f.height;
    res->n_bands = f.n_bands;
    memcpy(res->band_y0, f.band_y0, sizeof(f.band_y0));
    memcpy(res->band_y1, f.band_y1, sizeof(f.band_y1));
    res->x_min = f.x_min; res->x_max = f.x_max;
    res->n_obstacles = f.n_obstacles; res->n_gaps = f.n_gaps;
    memcpy(res->obstacles, f.obstacles, sizeof(f.obstacles));
    memcpy(res->gaps, f.gaps, sizeof(f.gaps));
    res->player_valid = f.player_valid;
    memcpy(res->player, f.player, sizeof(f.player));
    res->blob_x0 = f.blob_x0; res->blob_x1 = f.blob_x1;
    res->blob_y0 = f.blob_y0; res->blob_y1 = f.blob_y1; res->blob_area = f.blob_area;
    res->n_labels = f.n_labels;

    size_t bytes = vr_frame_size(f.n_obstacles, f.n_gaps);
    res->frame_bytes = bytes;
    if (frame_out && frame_cap >= bytes) vr_encode_frame(&f, frame_out);

    free(gray); free(wall); free(bright); free(rowc); free(colc);
    return 0;
}

int vr_ref_process(const uint8_t *rgb, int w, int h, int color,
                   uint8_t *gray_out, uint8_t *wall_out, uint8_t *bright_out,
                   uint16_t *row_count_out,
                   VrRefResult *res, uint8_t *frame_out, size_t frame_cap)
{
    return vr_ref_process_ex(rgb, w, h, color, gray_out, wall_out, bright_out,
                             row_count_out, res, frame_out, frame_cap,
                             (const VrRefParams *)0, 0, 0);
}
