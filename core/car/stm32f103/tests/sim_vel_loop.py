"""证据脚本（2026-10-29）：只搬 car_control.c 的速度环 + 最简轮子模型，
证明 diff 模式（bal=0）下厂家"正反馈"速度环会发散 —— 这就是
"@MODE 1 之后按哪个方向键两轮都反转"的根因。数字与 tests/test_car.c 一致。

跑：python3 core/car/stm32f103/tests/sim_vel_loop.py
轮子模型：pwm -> 计数/周期（10 ms），一阶滞后；死区 400。"""
KP_SPEED, KI_SPEED, INT_LIM, ALPHA = 50.0, 0.25, 15000.0, 0.3
ENC_PER_MPS, DEADZONE, MAXPWM = 200.0, 400.0, 6900.0
K_PLANT = 0.02          # 计数/周期 每 PWM（≈1300 PWM 对应 0.13 m/s = 26 计数）
LAG = 0.15              # 轮子一阶滞后 s

def run(v_cmd_mps, turn_pwm, blocked=False, secs=3.0):
    intg = lpf = 0.0
    wl = wr = 0.0
    t = 0.0
    rows = []
    dt = 0.01
    while t <= secs + 1e-9:
        movement = v_cmd_mps * ENC_PER_MPS
        intg += lpf - movement
        intg = max(-INT_LIM, min(INT_LIM, intg))
        vel = lpf * KP_SPEED + intg * KI_SPEED
        ml = max(-MAXPWM, min(MAXPWM, vel - turn_pwm))
        mr = max(-MAXPWM, min(MAXPWM, vel + turn_pwm))
        pl = (abs(ml) + DEADZONE) * (1 if ml >= 0 else -1)
        pr = (abs(mr) + DEADZONE) * (1 if mr >= 0 else -1)
        if blocked:
            sl = sr = 0.0
        else:
            sl = K_PLANT * (abs(pl) - DEADZONE) * (1 if pl >= 0 else -1)
            sr = K_PLANT * (abs(pr) - DEADZONE) * (1 if pr >= 0 else -1)
        a = dt / LAG
        wl += a * (sl - wl); wr += a * (sr - wr)
        lpf += ALPHA * ((wl + wr) - lpf)
        if abs(t * 1000 - round(t * 1000)) < 1e-6 and round(t * 1000) % 200 == 0:
            rows.append((t, intg, lpf, vel, pl, pr, wl, wr))
        t += dt
    return rows

print("=== @VEL 130 -600（v=0.13 m/s，差速 turn=-504）—— 轮子能转 ===")
print(" t(s)  int   lpf   vel   pwm_l pwm_r  wheel_l wheel_r  结论")
for t, intg, lpf, vel, pl, pr, wl, wr in run(0.13, -504):
    both = "两轮正转" if pl > 0 and pr > 0 else ("两轮反转" if pl < 0 and pr < 0 else "一正一反(原地转)")
    print("%5.1f %6.0f %5.1f %6.0f %5.0f %5.0f %7.1f %7.1f  %s" % (t, intg, lpf, vel, pl, pr, wl, wr, both))

print()
print("=== 同一指令，但轮子被纸片/摩擦卡住（编码器读数≈0）===")
print(" t(s)  int   lpf   vel   pwm_l pwm_r  结论")
for t, intg, lpf, vel, pl, pr, wl, wr in run(0.13, -504, blocked=True):
    both = "两轮正转" if pl > 0 and pr > 0 else ("两轮反转" if pl < 0 and pr < 0 else "一正一反(原地转)")
    print("%5.1f %6.0f %5.1f %6.0f %5.0f %5.0f  %s" % (t, intg, lpf, vel, pl, pr, both))
