# 板子 ↔ 小车：E30-433T20D 无线链路（配置流程 + 供电结论）

## 0. 先说结论：供电必须独立，别从板子 USB 取

实测（2026-10-26，板子 `/dev/video0` 换成接 USB 延长线后）：

```
usb 1-1-port1: attempt power cycle
usb 1-1-port1: Cannot enable. Maybe the USB cable is bad?
usb 1-1-port1: unable to enumerate USB device
dwc2 e0045000.usb: Overcurrent change detected      ← 过流
usb 1-1: USB disconnect, device number 2
```

⇒ 这块板子的 USB 口**过流保护已经被触发过**，与厂商"USB 口不支持供电"一致。
**E30 的 VCC 不要从板子 USB 取**，走独立 5 V + 共地。

## 1. 拓扑与接线

```
板子 PS UART1 (3.3V TTL) ── E30#1 ──(433MHz 空中)── E30#2 ── 小车 MCU UART
        ↑独立 5V + 共地                                ↑小车 5V
   USB-TTL 只在**配置/台架抓包**时用，不参与闭环
```

| 板子侧 | E30#1 | 说明 |
|---|---|---|
| UART TX（3.3V） | RXD | 交叉 |
| UART RX（3.3V） | TXD | 交叉 |
| GND | GND | **必须共地**（独立电源时这条尤其关键） |
| — | VCC | **独立 5 V**（≥200 mA 余量），模块端并 100 µF 电解 + 0.1 µF 瓷片 |
| 可选 GPIO | AUX | AUX 高 = 模块忙，别灌数据 |
| — | M0 / M1 | 透明模式：按手册接（多数 E3x：M0=0、M1=0）；配置模式：M0=1、M1=1 |

车侧同理，E30#2 的 UART 接小车 MCU，VCC 从小车 5 V 取。

> 电平：E30 是 **3.3 V TTL**；PS 的 UART 也是 3.3 V，**直连**。别接 RS232（±12 V）。

## 2. 配置流程（只有一根 USB-TTL 也能配好两块）

```bash
# ① USB-TTL 插 PC（WSL 里是 /dev/ttyUSB0）
ls -l /dev/ttyUSB0

# ② 模块跳线到"配置模式"（M0=1、M1=1，按手册），插上 USB-TTL
python3 core/board/tools/e30_config.py --port /dev/ttyUSB0 read

# ③ 写参数（信道/地址/空中速率/串口波特率），两块必须一模一样
python3 core/board/tools/e30_config.py --port /dev/ttyUSB0 write \
    --channel 0x17 --addr 0x0000 --air-rate 9.6k --uart-baud 9600

# ④ 保存一份，第二块照它核对
python3 core/board/tools/e30_config.py --port /dev/ttyUSB0 copy /tmp/e30.json

# ⑤ 换第二块 → 用同一组参数写 → 再 read 核对两份一致
# ⑥ 两块都跳回"透明传输"电平（M0=0、M1=0），装到板子/小车上
```

**参数怎么选**：我们的业务流量只有 ~750 B/s（`@POSE` 25 条/s × 30 B），
空中速率 **9.6k** 档足够且半双工冲突少；信道避开现场干扰（先扫一遍再定）。

## 3. 软件侧（已就位，只换传输层，协议不动）

* `core/board/tools/serial_link.py`：`SerialLink`，接口与现有 TCP `Link` 一致
  （`get_pose()` / `send_move()`），带 **seq + ACK + 重发 3 次 + AUX 流控**；
* `board_agent.py` 计划加 `--link tcp|serial`（**默认 tcp，基线不变**），串口分支只是把
  `Link` 换成 `SerialLink`；
* 安全层：车端 **1 s 收不到合法指令就停**；板端重发 3 次失败 → 上报断链 → 停车。

## 4. 台架联调顺序（不上车）

1. 两块模块各自上电，串口对串口打通：`echo hello | tee /dev/ttyUSB0` ↔ 对端读；
2. 用 `serial_link.py` 跑 10 Hz 心跳，测**往返时延**与**丢包率**（目标：<150 ms、<2%）；
3. 再把 `@MOVE`/`@ACK` 接进去，验证"半双工一问一答"不打架；
4. 最后才上闭环。

## 5. 已知风险

| 风险 | 对策 |
|---|---|
| 半双工冲突（两边同时发） | 一问一答 + ACK 超时重发，禁止多命令在飞 |
| 空中丢包 | seq + 重发 + 看门狗（车载 1 s 超时停车） |
| 模块缓冲溢出（AUX 高还灌） | 读 AUX 再发；或严格限制发送速率 |
| 433 MHz 干扰 / 金属遮挡 | 换信道、天线外置、抬高安装 |
| 与相机抢电（同一条 5V） | **独立供电**（见 §0） |
