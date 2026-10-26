#!/usr/bin/env python3
"""serial_link.py —— 板子↔小车 的串口链路（E30-433T20D 透明传输用）

接口与 `board_agent.py` 里现有的 TCP `Link` 完全一致（get_pose / send_move），
所以接上去只需要在 agent 里加一个 `--link serial` 分支，**基线逻辑一行不改**。

设计要点（射频频段必须接受的三件事）：
  1. **半双工**：模块同一时刻只能一个方向。这里用"命令—应答"配对 + 超时重发，
     不允许两条命令同时在飞；
  2. **会丢包**：每条报文带自增 seq，对端回 `@ACK <seq>`；重发 3 次仍失败就
     上报"链路断开"，由调用方（安全层）决定停车；
  3. **AUX 流控**：E30 的 AUX 高 = 模块忙，此时**不许再灌数据**（会丢）。
     AUX 若接到了 GPIO 就传 `aux_pin`；没接就靠"发送速率 < 空中速率"这条纪律。
"""
from __future__ import annotations

import threading
import time


class SerialLink:
    def __init__(self, port: str, baud: int = 115200, *, retries: int = 3,
                 ack_timeout: float = 0.3, aux_pin=None):
        import serial
        self._s = serial.Serial(port, baud, timeout=0.05)
        self._retries = int(retries)
        self._ack_to = float(ack_timeout)
        self._aux = aux_pin
        self._seq = 0
        self._acked = set()
        self._lock = threading.Lock()
        self._pose = (320.0, 60.0, 10.0)
        self._pose_seq = 0
        self._t_pose = time.monotonic()
        self._stop = False
        threading.Thread(target=self._rx_loop, daemon=True).start()

    # ---- 发送：一条一条来，带 seq 与重发 ----
    def _wait_idle(self, timeout: float = 0.05) -> None:
        """AUX 高的意思是模块还有数据没发完，别灌。"""
        if self._aux is None:
            return
        t0 = time.monotonic()
        while self._aux.value and time.monotonic() - t0 < timeout:
            time.sleep(0.001)

    def send(self, text: str) -> bool:
        with self._lock:
            self._seq += 1
            seq = self._seq
            line = f"{text.rstrip()}#{seq}\n"
            for _ in range(self._retries):
                self._wait_idle()
                self._s.write(line.encode())
                self._s.flush()
                t0 = time.monotonic()
                while time.monotonic() - t0 < self._ack_to:
                    if seq in self._acked:
                        self._acked.discard(seq)
                        return True
                    time.sleep(0.005)
            return False

    def send_move(self, act: int) -> bool:
        return self.send(f"@MOVE {act}")

    # ---- 接收：@POSE / @ACK ----
    def _rx_loop(self) -> None:
        buf = b""
        while not self._stop:
            try:
                buf += self._s.read(256) or b""
            except Exception:
                time.sleep(0.05)
                continue
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                s = line.decode("utf-8", "ignore").strip()
                if s.startswith("@ACK"):
                    p = s.split()
                    if len(p) >= 2 and p[1].isdigit():
                        self._acked.add(int(p[1]))
                elif s.startswith("@POSE"):
                    p = s.split()
                    if len(p) >= 5:
                        self._pose = (float(p[2]), float(p[3]), float(p[4]))
                        self._pose_seq = int(p[1])
                        self._t_pose = time.monotonic()

    def get_pose(self):
        return self._pose

    def get_pose_meta(self):
        return self._pose_seq, self._t_pose

    def close(self) -> None:
        self._stop = True
        try:
            self._s.close()
        except Exception:
            pass
