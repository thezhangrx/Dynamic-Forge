/* 诊断：AXI 通路是否可用 + 视觉流水卡在哪一步。
 *
 * 寄存器映射（core/fpga/rtl/vision_axi.v）：
 *   0x000+4*i  配置寄存器（可读可写）
 *   0x080      调试选择（写）：{4'b0, sub[3:0], idx[17:0], sel[3:0]}
 *   0x084      调试数据（读）
 *   0x0C0      控制/状态
 *
 * 调试口的 sel/sub 含义见 vision_top.v 的 dbg_sel 表；
 * 这里的子命令取 sel=7 的状态项（n_bands/x 范围/n_obs/n_gaps/n_labels/n_segs）。
 */
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define BASE 0x43C00000UL
static volatile uint32_t *R;

static void w(uint32_t off, uint32_t v) { *(volatile uint32_t *)((volatile char *)R + off) = v; }
static uint32_t r(uint32_t off) { return *(volatile uint32_t *)((volatile char *)R + off); }

int main(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    R = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, BASE);
    if (R == MAP_FAILED) { perror("mmap"); return 1; }

    printf("=== ① AXI 通路自检：写配置寄存器再读回 ===\n");
    for (int i = 0; i < 3; i++) {
        uint32_t off = 4u * (uint32_t)i;
        uint32_t before = r(off);
        w(off, 0xDEAD0000u + (uint32_t)i);
        uint32_t after = r(off);
        printf("  0x%03X: 读回 0x%08X  %s\n", off, after,
               (after == 0xDEAD0000u + (uint32_t)i) ? "✓ 通" : "✗ 不通");
        (void)before;
    }

    printf("\n=== ② 状态寄存器 ===\n");
    printf("  0x0C0 ctrl=0x%08X (bit0 busy=%u, bit1 frame_done=%u)\n",
           r(0x0C0), r(0x0C0) & 1u, (r(0x0C0) >> 1) & 1u);
    printf("  0x0D0 flen=%u\n", r(0x0D0));

    printf("\n=== ③ 调试口：流水线内部状态（sel=7）===\n");
    const char *nm[7] = {"n_bands", "x_min", "x_max", "n_obs", "n_gaps", "n_labels", "n_segs"};
    for (uint32_t sub = 0; sub <= 6; sub++) {
        w(0x080, (sub << 22) | (0u << 4) | 7u);
        usleep(1000);
        printf("  %-8s = %u\n", nm[sub], r(0x084));
    }

    printf("\n=== ④ 调试口：帧存抽样（sel=0/1/2, idx=0 与 idx=100）===\n");
    for (uint32_t sel = 0; sel < 3; sel++) {
        for (uint32_t idx = 0; idx <= 100; idx += 100) {
            w(0x080, (0u << 22) | (idx << 4) | sel);
            usleep(1000);
            printf("  sel=%u idx=%3u -> 0x%08X\n", sel, idx, r(0x084));
        }
    }

    printf("\n=== ⑤ 复位并观察 busy 是否变化（10 ms 内采样）===\n");
    for (int i = 0; i < 5; i++) { printf("  t=%d ctrl=0x%08X\n", i, r(0x0C0)); usleep(2000); }
    return 0;
}
