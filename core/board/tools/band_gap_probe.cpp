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
    for (int i = 1; i < argc; ++i) {
        std::vector<uint8_t> rgb;
        int w = 0, h = 0;
        if (read_ppm(argv[i], &rgb, &w, &h) != 0) {
            std::printf("%s ERR\n", argv[i]);
            continue;
        }
        const band_gap::Door d = band_gap::find(rgb.data(), w, h);
        std::printf("%s found=%d bands=%d y0=%d y1=%d x0=%d x1=%d cx=%d w=%d "
                    "support=%.2f snr=%.1f\n",
                    argv[i], d.found ? 1 : 0, d.n_bands, d.y0, d.y1,
                    d.x0, d.x1, d.cx, d.width, d.support, d.snr);
    }
    return 0;
}
