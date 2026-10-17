/* ===========================================================================
 * band_gap_probe.cpp —— 把 `band_gap.hpp`（结构判据）跑在一张 P6 PPM 上，打一行结果。
 *
 * 用途有两个：
 *   1. 离线看真机位帧上"到底哪条带、缺口多宽、支撑比多少"（调判据时不靠猜）；
 *   2. 与 `core/vision/relative.py` 的 `best_door()` **逐项对账** ——
 *      `tests/test_band_gap_cpp.py` 就是拿它编译后比两边的。
 *
 * 编译（PC 上就够，不需要板子）：
 *     g++ -std=c++17 -O2 -I core/board -o /tmp/band_gap_probe \
 *         core/board/tools/band_gap_probe.cpp
 * 运行：
 *     /tmp/band_gap_probe /tmp/f_70_12.ppm
 *     /tmp/band_gap_probe data/fpga/run4_720to360/*.ppm | head
 * =========================================================================== */
#include "band_gap.hpp"

#include <cstdio>
#include <cstdint>
#include <string_view>
#include <vector>

namespace {

int read_ppm(const char* path, std::vector<uint8_t>* rgb, int* w, int* h) {
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return 1;
    char magic[3] = {0};
    if (std::fscanf(f, "%2s", magic) != 1 || magic[0] != 'P' || magic[1] != '6') {
        std::fclose(f);
        return 2;
    }
    long vals[2] = {0, 0};
    int nv = 0;
    while (nv < 2) {
        const int c = std::fgetc(f);
        if (c == '#') { while (c != '\n' && c != EOF) std::fgetc(f); continue; }
        if (c == EOF) break;
        if (c >= '0' && c <= '9') {
            std::ungetc(c, f);
            if (std::fscanf(f, "%ld", &vals[nv]) == 1) ++nv;
        }
    }
    int maxv = 0;
    if (std::fscanf(f, "%d", &maxv) != 1) { std::fclose(f); return 3; }
    std::fgetc(f);
    *w = static_cast<int>(vals[0]);
    *h = static_cast<int>(vals[1]);
    rgb->resize(static_cast<size_t>(*w) * static_cast<size_t>(*h) * 3u);
    const size_t got = std::fread(rgb->data(), 1, rgb->size(), f);
    std::fclose(f);
    return got == rgb->size() ? 0 : 4;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法: %s <a.ppm> [b.ppm ...]\n", argv[0]);
        return 2;
    }
    // `--angle0`：关掉角度搜索（只按 0° 跑）。用于与 `relative.py` 的
    // `best_door()` 逐项对账 —— 那边是固定角度，两边必须在**同一条件**下比。
    int first = 1;
    const bool angle0 = (argc > 1 && std::string_view(argv[1]) == "--angle0");
    if (angle0) first = 2;
    for (int i = first; i < argc; ++i) {
        std::vector<uint8_t> rgb;
        int w = 0, h = 0;
        if (read_ppm(argv[i], &rgb, &w, &h) != 0) {
            std::printf("%s ERR\n", argv[i]);
            continue;
        }
        const band_gap::Door d = angle0
            ? band_gap::find_at(rgb.data(), w, h, 0.0)
            : band_gap::find(rgb.data(), w, h);
        const band_gap::Extent e = band_gap::field_extent(rgb.data(), w, h);
        std::printf("%s found=%d bands=%d y0=%d y1=%d x0=%d x1=%d cx=%d w=%d "
                    "support=%.2f snr=%.1f\n",
                    argv[i], d.found ? 1 : 0, d.n_bands, d.y0, d.y1,
                    d.x0, d.x1, d.cx, d.width, d.support, d.snr);
        std::printf("    extent ok=%d x=%d..%d y=%d..%d w_px=%.0f h_px=%.0f "
                    "scale=%.3f/%.3f aniso=%.2f\n",
                    e.ok ? 1 : 0, e.x0, e.x1, e.y0, e.y1, e.width_px, e.height_px,
                    e.scale_x, e.scale_y, e.anisotropy);
    }
    return 0;
}
