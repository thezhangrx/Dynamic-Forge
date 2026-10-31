/* ===========================================================================
 * dump_cc.c —— 把 C 参考模型的**候选连通域全表**打出来。
 *
 * 为什么需要它
 * ------------
 * `vr_ref_process_ex()` 只回**被选中的**那个连通域（`blob_*` / `player_*`），
 * 看不见全表。真实照片语料上 RTL 选中了 area=134 的 blob 而 C 一个都没选，
 * 光看接口分不清是**表算错了**还是**判据判错了**。
 *
 * 做法：`#include "vision_ref.c"`（配 `-DVR_NO_MAIN`），于是 `vr_comp`、
 * `uf_parent`、`uf_find` 这些文件作用域的 static 都能直接读 —— 拿到的就是
 * 参考模型真正的中间状态，不是复制品。
 *
 * 用法::
 *
 *     gcc -O2 -DVR_NO_MAIN -I. -o dump_cc dump_cc.c
 *     ./dump_cc ../../../data/fpga/stim_photo/000002.ppm
 *
 * RTL 侧的对应视图由 `tb_vision.cpp` 的 `--dump-cc` 给出（读 CCL 的
 * `parent/cmp_*`），两边的行可以逐列 diff。见 docs/帧存改造.md §14.4。
 * =========================================================================== */
#define VR_NO_MAIN
#include "vision_ref.c"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* P6 PPM 读取（与 tb_vision.cpp 的 load_ppm 同格式） */
static int load_ppm(const char *path, int *w, int *h, unsigned char **rgb)
{
    FILE *fp = fopen(path, "rb");
    if (!fp) return 0;
    char magic[3] = {0};
    if (fscanf(fp, "%2s", magic) != 1 || strcmp(magic, "P6") != 0) { fclose(fp); return 0; }
    int vals[2] = {0, 0}, n = 0, maxv = 0, c, seen = 0;
    while (n < 2 && (c = fgetc(fp)) != EOF) {
        if (c == '#') { while ((c = fgetc(fp)) != EOF && c != '\n') {} continue; }
        if (c >= '0' && c <= '9') vals[n] = vals[n] * 10 + (c - '0');
        else if (vals[n] > 0) n++;
    }
    while ((c = fgetc(fp)) != EOF) {
        if (c >= '0' && c <= '9') maxv = maxv * 10 + (c - '0');
        else if (maxv > 0) { seen = 1; break; }
    }
    if (!seen || maxv != 255) { fclose(fp); return 0; }
    *w = vals[0]; *h = vals[1];
    *rgb = (unsigned char *)malloc((size_t)(*w) * (*h) * 3);
    if (!*rgb) { fclose(fp); return 0; }
    if (fread(*rgb, 1, (size_t)(*w) * (*h) * 3, fp) != (size_t)(*w) * (*h) * 3) {
        fclose(fp); return 0;
    }
    fclose(fp);
    return 1;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "用法: %s <x.ppm>\n", argv[0]); return 2; }
    int w = 0, h = 0;
    unsigned char *rgb = NULL;
    if (!load_ppm(argv[1], &w, &h, &rgb)) { fprintf(stderr, "读不了 %s\n", argv[1]); return 1; }

    size_t n = (size_t)w * h;
    unsigned char *gray = (unsigned char *)malloc(n);
    unsigned char *wall = (unsigned char *)malloc(n);
    unsigned char *bright = (unsigned char *)malloc(n);
    unsigned short *rowc = (unsigned short *)malloc(sizeof(unsigned short) * (size_t)h);
    VrRefResult res;
    VrRefParams p;
    memset(&res, 0, sizeof(res));
    vr_ref_default_params(&p);
    if (vr_ref_process_ex(rgb, w, h, 1, gray, wall, bright, rowc, &res, NULL, 0,
                          &p, 0, 0) != 0) {
        fprintf(stderr, "参考模型失败\n");
        return 1;
    }

    printf("帧 %dx%d  n_labels=%d  player_valid=%d  blob=(%d,%d,%d,%d) area=%d\n",
           w, h, res.n_labels, res.player_valid,
           res.blob_x0, res.blob_x1, res.blob_y0, res.blob_y1, res.blob_area);
    printf("阈值 min_area=%d max_area=%d fill=%d%% aspect=%d%%\n",
           p.player_min_area, p.player_max_area,
           p.player_min_fill_pct, p.player_max_aspect_pct);
    printf("%-6s %-8s %-7s %-7s %-7s %-7s %-8s %-7s %s\n",
           "L", "n", "x0", "x1", "y0", "y1", "wh", "fill%", "判据(A/E/F/S)");

    int best = -1;
    long long best_num = 0, best_den = 1;
    for (int i = 0; i < res.n_labels; i++) {
        if (uf_find(i) != i) continue;
        const VrComp *c = &vr_comp[i];
        int ww = c->x1 - c->x0 + 1, hh = c->y1 - c->y0 + 1;
        long wh = (long)ww * (long)hh;
        int a_ok = (c->n >= p.player_min_area && c->n <= p.player_max_area);
        int e_ok = (c->x0 > 0 && c->y0 > 0 && c->x1 < w - 1 && c->y1 < h - 1);
        int f_ok = (100L * c->n >= (long)p.player_min_fill_pct * wh);
        int mx = (ww > hh) ? ww : hh, mn = (ww > hh) ? hh : ww;
        int s_ok = (100L * mx <= (long)p.player_max_aspect_pct * mn);
        int pass = a_ok && e_ok && f_ok && s_ok;
        printf("%-6d %-8d %-7d %-7d %-7d %-7d %-8ld %-7.1f %c%c%c%c%s\n",
               i, c->n, c->x0, c->x1, c->y0, c->y1, wh,
               wh ? 100.0 * c->n / (double)wh : 0.0,
               a_ok ? 'A' : '-', e_ok ? 'E' : '-', f_ok ? 'F' : '-', s_ok ? 'S' : '-',
               pass ? "  <== 通过" : "");
        if (pass) {
            long long num = (long long)c->n * (long long)c->n;
            if (num * best_den > best_num * (long long)wh) {
                best_num = num; best_den = wh; best = i;
            }
        }
    }
    printf("C 选中的 best=%d\n", best);
    return 0;
}
