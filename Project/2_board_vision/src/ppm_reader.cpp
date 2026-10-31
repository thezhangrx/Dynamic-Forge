// ===========================================================================
// ppm_reader.cpp —— 见 ppm_reader.hpp
// ===========================================================================
#include "ppm_reader.hpp"

#include <cstdio>
#include <fstream>

namespace vision_board {
namespace {

//: 跳过空白与 `#` 注释，取下一个整数。PPM 头允许任意空白/注释分隔。
bool next_int(std::istream& in, int* out) {
    int c = in.peek();
    for (;;) {
        while (c != EOF && (c == ' ' || c == '\t' || c == '\n' || c == '\r')) {
            in.get();
            c = in.peek();
        }
        if (c == '#') {                       // 注释到行尾
            while (c != EOF && c != '\n') { in.get(); c = in.peek(); }
            continue;
        }
        break;
    }
    if (c == EOF) return false;
    int v = 0;
    if (in >> v) { *out = v; return true; }
    return false;
}

}   // namespace

bool read_ppm_p6(const std::string& path, int* w, int* h,
                 std::vector<uint8_t>* rgb, std::string* why) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { *why = "打不开文件 " + path; return false; }

    char magic[3] = {0, 0, 0};
    f.read(magic, 2);
    if (!f || magic[0] != 'P' || magic[1] != '6') {
        *why = path + " 不是 P6 PPM（只支持 P6 / maxval 255，不做静默降级）";
        return false;
    }
    int iw = 0, ih = 0, maxv = 0;
    if (!next_int(f, &iw) || !next_int(f, &ih) || !next_int(f, &maxv)) {
        *why = path + " 的头读不出来（应该是 P6 <w> <h> <maxval>）";
        return false;
    }
    if (iw <= 0 || ih <= 0) { *why = path + " 的尺寸非法"; return false; }
    if (maxv != 255) {
        *why = path + " 的 maxval=" + std::to_string(maxv) + "，只支持 255";
        return false;
    }
    f.get();                                   // 头与数据之间恰好一个空白
    if (!f) { *why = path + " 的头之后没有数据"; return false; }

    const size_t need = static_cast<size_t>(iw) * static_cast<size_t>(ih) * 3u;
    rgb->resize(need);
    f.read(reinterpret_cast<char*>(rgb->data()), static_cast<std::streamsize>(need));
    if (static_cast<size_t>(f.gcount()) != need) {
        *why = path + " 的数据不够：要 " + std::to_string(need) + " 字节，只有 "
             + std::to_string(f.gcount());
        return false;
    }
    *w = iw;
    *h = ih;
    return true;
}

}   // namespace vision_board
