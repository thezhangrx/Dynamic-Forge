#include "bsp_motor.h"

/*
 * D153C 双路 TB6612 驱动板的 PWM 输出（引脚见 board.h 的说明）：
 *   A 电机：PB1 = TIM3_CH4   B 电机：PB0 = TIM3_CH3
 *   TIM3 是通用定时器，72 MHz / (7199+1) = 10 kHz，不需要 BDTR/MOE。
 *
 *   CCMR2: OC3M[2:0]=110(PWM1) 位于 bit6:4（CH3），OC4M 位于 bit14:12（CH4）
 *          （CH1/CH2 才在 CCMR1，别写错）
 *   CCER : CC3E(8) CC4E(12)
 *   CCR  : A -> CCR4，B -> CCR3（和厂家例程一致）
 *
 * 方向极性照厂家 Set_PWM()：
 *   A 正转 -> AIN1(PB14)=0, AIN2(PB15)=1
 *   B 正转 -> BIN1(PB13)=1, BIN2(PB12)=0
 * （两个电机在底盘上安装方向相反，所以极性相反）
 */

/* 极性（2026-10-24 实测确定）：两路命令"正转"时，电机要往同一个方向转。
 * 之前 A=1 / B=0 是照厂家例程抄的（厂家底盘电机镜像安装），
 * 但实测两个轮子反向 —— 改成同一极性，以"两轮同向"为准。 */
#define MOT_A_INVERT 1          /* A 电机：正转时 AIN1 为低（AIN2 高） */
#define MOT_B_INVERT 1          /* B 电机：正转时 BIN1 为低（BIN2 高） */

static int16_t clamp1000(int16_t v)
{
    if (v >  MOTOR_OUT_MAX) return  MOTOR_OUT_MAX;
    if (v < -MOTOR_OUT_MAX) return -MOTOR_OUT_MAX;
    return v;
}

static void motor_gpio_init(void)
{
    RCC->APB2ENR |= (1u << 3);                       /* IOPBEN */

    /* PWM 脚：复用推挽 50 MHz（PB0/PB1） */
    gpio_cfg(MOT_A_PWM_PORT, MOT_A_PWM_PIN, GPIO_CFG_AF_PP_50);
    gpio_cfg(MOT_B_PWM_PORT, MOT_B_PWM_PIN, GPIO_CFG_AF_PP_50);

    /* 方向脚：推挽 50 MHz，先拉低 */
    gpio_cfg(MOT_AIN1_PORT, MOT_AIN1_PIN, GPIO_CFG_OUT_PP_50);
    gpio_cfg(MOT_AIN2_PORT, MOT_AIN2_PIN, GPIO_CFG_OUT_PP_50);
    gpio_cfg(MOT_BIN1_PORT, MOT_BIN1_PIN, GPIO_CFG_OUT_PP_50);
    gpio_cfg(MOT_BIN2_PORT, MOT_BIN2_PIN, GPIO_CFG_OUT_PP_50);

    gpio_set(MOT_AIN1_PORT, MOT_AIN1_PIN, 0);
    gpio_set(MOT_AIN2_PORT, MOT_AIN2_PIN, 0);
    gpio_set(MOT_BIN1_PORT, MOT_BIN1_PIN, 0);
    gpio_set(MOT_BIN2_PORT, MOT_BIN2_PIN, 0);
}

void motor_init(void)
{
    motor_gpio_init();

    RCC->APB1ENR |= (1u << 1);                       /* TIM3EN */

    TIM3->PSC = 0;
    TIM3->ARR = MOTOR_PWM_ARR;                       /* 7199 -> 10 kHz */
    TIM3->RCR = 0;

    /* CH3 / CH4 -> PWM 模式 1，使能预装载
     * ⚠ 通用定时器的 CH3/CH4 在 **CCMR2** 里（CH1/CH2 才在 CCMR1）：
     *   CCMR2.OC3M[6:4]  OC3PE[3]    -> CH3 (PB0, PWMB)
     *   CCMR2.OC4M[14:12] OC4PE[11]  -> CH4 (PB1, PWMA)
     * （之前错把 CH3 写进了 CCMR1，导致 PB0 一直不输出） */
    TIM3->CCMR2 = (TIM3->CCMR2 & ~((0x7u << 4) | (0x7u << 12)))
                | (0x6u << 4)  | (1u << 3)
                | (0x6u << 12) | (1u << 11);

    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    TIM3->CCER |= (1u << 8) | (1u << 12);            /* CC3E | CC4E */
    TIM3->CR1  |= (1u << 7) | (1u << 0);             /* ARPE | CEN */
}

/* 写一个轮子的方向和占空比。duty 直接是 CCR 计数（0..ARR） */
static void set_one(uint32_t ch, GPIO_TypeDef *p1, int n1,
                    GPIO_TypeDef *p2, int n2, int16_t v, uint32_t duty, int invert)
{
    int fwd_high_on_p1 = invert ? 0 : 1;

    if (v > 0) {
        gpio_set(p1, n1, fwd_high_on_p1);
        gpio_set(p2, n2, !fwd_high_on_p1);
    } else if (v < 0) {
        gpio_set(p1, n1, !fwd_high_on_p1);
        gpio_set(p2, n2, fwd_high_on_p1);
    } else {
        gpio_set(p1, n1, 0);
        gpio_set(p2, n2, 0);
    }

    if (duty > MOTOR_PWM_ARR) duty = MOTOR_PWM_ARR;
    if (ch == 0) TIM3->CCR4 = duty;                  /* A 电机 = CH4 */
    else         TIM3->CCR3 = duty;                  /* B 电机 = CH3 */
}

void motor_set(int16_t a, int16_t b)
{
    a = clamp1000(a);
    b = clamp1000(b);

    set_one(0, MOT_AIN1_PORT, MOT_AIN1_PIN, MOT_AIN2_PORT, MOT_AIN2_PIN, a,
            (uint32_t)(a < 0 ? -a : a) * MOTOR_PWM_ARR / MOTOR_OUT_MAX, MOT_A_INVERT);
    set_one(1, MOT_BIN1_PORT, MOT_BIN1_PIN, MOT_BIN2_PORT, MOT_BIN2_PIN, b,
            (uint32_t)(b < 0 ? -b : b) * MOTOR_PWM_ARR / MOTOR_OUT_MAX, MOT_B_INVERT);
}

void motor_set_pwm(int16_t a, int16_t b)
{
    int32_t lim = (int32_t)MOTOR_PWM_ARR;
    int32_t pa = a, pb = b;

    if (pa >  lim) pa =  lim;
    if (pa < -lim) pa = -lim;
    if (pb >  lim) pb =  lim;
    if (pb < -lim) pb = -lim;

    set_one(0, MOT_AIN1_PORT, MOT_AIN1_PIN, MOT_AIN2_PORT, MOT_AIN2_PIN, (int16_t)pa,
            (uint32_t)(pa < 0 ? -pa : pa), MOT_A_INVERT);
    set_one(1, MOT_BIN1_PORT, MOT_BIN1_PIN, MOT_BIN2_PORT, MOT_BIN2_PIN, (int16_t)pb,
            (uint32_t)(pb < 0 ? -pb : pb), MOT_B_INVERT);
}

void motor_stop(void)
{
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    gpio_set(MOT_AIN1_PORT, MOT_AIN1_PIN, 0);
    gpio_set(MOT_AIN2_PORT, MOT_AIN2_PIN, 0);
    gpio_set(MOT_BIN1_PORT, MOT_BIN1_PIN, 0);
    gpio_set(MOT_BIN2_PORT, MOT_BIN2_PIN, 0);
}

void motor_brake(void)
{
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
    gpio_set(MOT_AIN1_PORT, MOT_AIN1_PIN, 1);
    gpio_set(MOT_AIN2_PORT, MOT_AIN2_PIN, 1);
    gpio_set(MOT_BIN1_PORT, MOT_BIN1_PIN, 1);
    gpio_set(MOT_BIN2_PORT, MOT_BIN2_PIN, 1);
}
