/* ===========================================================================
 * read_frame.c —— 在**板子上**读回几何帧，并和协同仿真的期望值逐位对账
 *
 * 这是"真硅上整条链对不对"的第一次端到端验证：
 *
 *     tpg_source（片上图案）→ vision_axi（视觉流水）→ AXI 窗口 → 本程序 → 对账
 *
 * 期望值从哪来：`core/fpga/cosim/tb_tpg.cpp` 里 Vvision_selftest 逐场景跑出来的
 * 帧长与 CRC32（见 core/fpga/docs/板上验证.md §2.2）。块设计里的图案源就是同一个
 * `vision_tpg`、相同的默认配置，所以这两组常数**应当完全相同**。
 *
 * 编译（在板子上直接编）：
 *     gcc -O2 -o read_frame read_frame.c
 * 运行：
 *     ./read_frame            # 读一帧并对账
 *     ./read_frame 40         # 连续读 40 帧，统计命中率
 *
 * 寄存器映射见 core/fpga/rtl/vision_axi.v：
 *     基址 + 0x0C0  控制/状态（bit0 busy, bit1 frame_done）
 *     基址 + 0x0D0  上一帧字节数（只读）
 *     基址 + 0x100  几何帧窗口
 * =========================================================================== */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define BASE        0x43C00000UL
#define MAP_SIZE    0x1000UL
#define OFF_CTRL    0x0C0
#define OFF_FLEN    0x0D0
#define OFF_FRAME   0x100
#define FRAME_MAX   2048

/* 期望值（来自协同仿真的 Vvision_selftest，图案 0..7） */
static const uint32_t EXP_CRC[8] = {
    0xA9670ED9u, 0x621801BEu, 0x1EEBDE91u, 0x0BC5C6CAu,
    0x76D2BFA9u, 0xC71A351Au, 0xA5440B40u, 0xAEC7F26Eu,
};
#define EXP_LEN 184

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++)
            c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1u)));
    }
    return ~c;
}

int main(int argc, char **argv)
{
    int want = (argc > 1) ? atoi(argv[1]) : 1;

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    volatile uint8_t *m = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, BASE);
    if (m == MAP_FAILED) { perror("mmap"); return 1; }

    volatile uint32_t *ctrl = (volatile uint32_t *)(m + OFF_CTRL);
    volatile uint32_t *flen = (volatile uint32_t *)(m + OFF_FLEN);
    const uint8_t *fp = (const uint8_t *)(m + OFF_FRAME);

    printf("基址 0x%08lX  ctrl=0x%08X  flen=%u\n",
           (unsigned long)BASE, (unsigned)*ctrl, (unsigned)*flen);

    int ok = 0, bad = 0, seen[8] = {0};
    uint8_t buf[FRAME_MAX];

    for (int f = 0; f < want; f++) {
        /* 等一个 frame_done（bit1），最多 200 ms */
        uint32_t st0 = *ctrl;
        int waited = 0;
        while (waited < 200) {
            uint32_t st = *ctrl;
            if ((st & 2u) != (st0 & 2u)) break;      /* 翻转即新的一帧 */
            usleep(1000);
            waited++;
        }
        uint32_t len = *flen;
        if (len == 0 || len > FRAME_MAX) { printf("帧 %d: 长度异常 %u\n", f, len); bad++; continue; }
        memcpy(buf, (const void *)fp, len);          /* 快照，避免读到一半被改写 */

        uint32_t crc = crc32(buf, len);
        int pat = -1;
        for (int i = 0; i < 8; i++)
            if (len == EXP_LEN && crc == EXP_CRC[i]) { pat = i; break; }

        if (pat >= 0) { ok++; seen[pat]++; }
        else          { bad++; }

        printf("帧 %2d: len=%4u crc=0x%08X  %s\n", f, len, crc,
               pat >= 0 ? "✓ 与协同仿真一致" : "✗ **不符**");

        if (pat < 0 && bad <= 2) {                   /* 头几次不符时打出头部，便于定位 */
            printf("       头 24 字节:");
            for (int i = 0; i < 24 && i < (int)len; i++) printf(" %02X", buf[i]);
            printf("\n");
        }
    }

    printf("\n=== 结果：一致 %d / %d，不符 %d ===\n", ok, want, bad);
    printf("图案命中分布:");
    for (int i = 0; i < 8; i++) printf(" [%d]=%d", i, seen[i]);
    printf("\n");
    return bad ? 2 : 0;
}
