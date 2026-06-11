#!/usr/bin/env python3
"""
test_gripper_sine.py
--------------------
Streams a sine wave position to the WSG32 gripper using GCL text protocol.
Tests whether the gripper can track continuous position changes smoothly.

Prerequisites:
  - Text interface enabled: http://192.168.1.201 → Settings → Command Interface
    → USE text based interface = ON
  - Gripper powered and homed

Run:
  python3 test_gripper_sine.py

Controls:
  Ctrl+C to stop
"""

import socket
import time
import math
import threading

GRIPPER_IP   = "192.168.1.201"
GRIPPER_PORT = 1000
MIN_POS      = 5.0    # mm — don't fully close
MAX_POS      = 50.0   # mm
FREQ_HZ      = 0.3    # sine frequency
SPEED        = 200    # mm/s — gripper jaw speed
CMD_HZ       = 20     # command rate

class WSGSineTest:
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(5.0)
        self._lock = threading.Lock()
        self._last_dir = 0.0
        self._last_pos = -1.0

    def connect(self):
        self._sock.connect((GRIPPER_IP, GRIPPER_PORT))
        print(f"Connected to {GRIPPER_IP}:{GRIPPER_PORT}")
        # Clear fault
        self._send("FSACK()")
        time.sleep(0.2)
        # Home
        print("Homing...")
        self._sock.settimeout(15.0)
        self._send_wait("HOME()", "FIN HOME")
        self._sock.settimeout(2.0)
        print("Homed. Starting sine wave...")

    def _send(self, cmd):
        with self._lock:
            self._sock.sendall((cmd + "\n").encode("ascii"))

    def _send_wait(self, cmd, expected):
        self._send(cmd)
        buf = b""
        deadline = time.time() + 15.0
        while time.time() < deadline:
            chunk = self._sock.recv(256)
            if not chunk:
                break
            buf += chunk
            if expected.encode() in buf:
                return True
        return False

    def move(self, pos_mm):
        pos_mm = max(MIN_POS, min(MAX_POS, pos_mm))

        # Only send STOP on direction reversal
        new_dir = pos_mm - self._last_pos
        if self._last_pos >= 0 and new_dir != 0 and self._last_dir != 0:
            if (new_dir > 0) != (self._last_dir > 0):
                self._send("STOP()")
                time.sleep(0.005)

        if new_dir != 0:
            self._last_dir = new_dir
        self._last_pos = pos_mm

        self._send(f"MOVE({pos_mm:.1f},{SPEED})")

    def run(self):
        dt = 1.0 / CMD_HZ
        t = 0.0
        amp = (MAX_POS - MIN_POS) / 2.0
        center = (MAX_POS + MIN_POS) / 2.0

        print(f"Streaming at {CMD_HZ}Hz | freq={FREQ_HZ}Hz | "
              f"range=[{MIN_POS},{MAX_POS}]mm | Ctrl+C to stop")

        while True:
            loop_start = time.time()

            pos = center + amp * math.sin(2 * math.pi * FREQ_HZ * t)
            self.move(pos)
            print(f"\r  t={t:6.2f}s  target={pos:5.1f}mm", end="", flush=True)

            t += dt
            elapsed = time.time() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def disconnect(self):
        try:
            self._send("STOP()")
            self._sock.close()
        except Exception:
            pass

def main():
    tester = WSGSineTest()
    try:
        tester.connect()
        tester.run()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        tester.disconnect()

if __name__ == "__main__":
    main()