"""
wsg_tcp.py
----------
Pure-Python GCL (Gripper Control Language) TCP driver for the Weiss WSG 32.

No ROS dependency — this layer only knows about sockets and the GCL protocol.
The ROS2 node (wsg32_node.py) imports this class.

PREREQUISITE (one-time hardware setup):
    Open http://<gripper_ip> in a browser.
    Go to Settings → Command Interface → enable "Use text based interface".
    This setting persists across reboots.

GCL protocol basics:
    - All commands are ASCII strings terminated with \\n
    - Gripper responds with "ACK <CMD>\\n" (immediate) or "FIN <CMD>\\n" (motion done)
    - On error: "ERR <CODE>\\n"
    - Default IP: 192.168.1.20  |  Default port: 1000

Adapted from: https://github.com/real-stanford/minWSG (MIT License)
Extended with: non-blocking move, GETSTATE feedback polling, connection guards.

State feedback design
---------------------
The WSG 32 accepts multiple simultaneous TCP connections on port 1000.
We use TWO connections:
  _sock        → command socket  (MOVE, HOME, FSACK, ...)
  _state_sock  → feedback socket (GETSTATE polling on a background thread)

This keeps command latency and state polling completely decoupled.
move_nonblocking() fires on _sock and returns in <1 ms.
The background thread reads _state_sock at poll_hz and updates _state.
The ROS node reads _state (thread-safe) and publishes measured position.
"""

import socket
import threading
from dataclasses import dataclass, field
from time import time, sleep


@dataclass
class GripperState:
    """Measured gripper state, updated by the background polling thread."""
    pos_mm:    float = 0.0   # measured jaw width in mm
    speed_mms: float = 0.0   # jaw velocity in mm/s
    force_n:   float = 0.0   # motor force in N
    state_str: str   = "UNKNOWN"
    timestamp: float = field(default_factory=time)  # time.time() of last update

    def is_fresh(self, max_age_s: float = 0.5) -> bool:
        """Returns False if no update has arrived recently — indicates poll failure."""
        return (time() - self.timestamp) < max_age_s


class WSG32TCP:
    """
    Thread-safe GCL TCP driver for the Weiss WSG 32 gripper.

    Key design decisions:
    - TCP_NODELAY on both sockets — kills Nagle algorithm, no buffering latency.
    - move_nonblocking() sends MOVE and returns without waiting for FIN.
      This is what you want during teleoperation: the leader keeps streaming
      new positions and the gripper tracks continuously.
    - move() is the blocking version — use it for scripted sequences.
    - A background thread polls GETSTATE on a dedicated second TCP connection,
      populating self.state with measured position, speed, and force.
      Read self.state from any thread — it is protected by a lock.
    """

    # Gripper physical limits for WSG 32
    MIN_POS_MM = 0.0    # fully closed
    MAX_POS_MM = 55.0   # fully open (WSG 32 stroke = 55 mm, NOT 110)
                        # NOTE: WSG 50 stroke is 110 mm. Verify with your hardware.
                        # Check Settings → System Info in the web UI.

    def __init__(self,
                 ip: str = "192.168.1.201",
                 port: int = 1000,
                 timeout: float = 5.0,
                 poll_hz: float = 10.0):
        """
        ip       : gripper IP on the ghost subnet
        port     : GCL TCP port (always 1000)
        timeout  : socket timeout for blocking calls (seconds)
        poll_hz  : GETSTATE polling rate for the feedback thread (Hz)
                   50 Hz is plenty for position-controlled teleoperation.
                   The gripper's internal controller runs faster regardless.
        """
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self._poll_interval = 1.0 / poll_hz

        # Command socket (main thread)
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()   # one command at a time on _sock

        # Feedback socket (background thread only — never touch from main thread)
        self._state_sock: socket.socket | None = None

        # Shared state struct — written by background thread, read by ROS node
        self._state = GripperState()
        self._state_lock = threading.Lock()

        # Background polling thread
        self._poll_thread: threading.Thread | None = None
        self._polling = False

        self._last_cmd_pos:      float | None = None
        self._last_gripper_dir:  float | None = None    

    # ──────────────────────────────────────────────────────────────────────
    # Connection
    # ──────────────────────────────────────────────────────────────────────

    @staticmethod
    def _make_socket(ip: str, port: int, timeout: float) -> socket.socket:
        """Open one TCP connection to the gripper with TCP_NODELAY."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # TCP_NODELAY: send each command immediately, do not wait to batch.
        # Without this you get random ~40 ms delays from Nagle buffering.
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(timeout)
        s.connect((ip, port))
        return s

    def connect(self) -> None:
        """
        Open TWO TCP connections (command + feedback) and start the
        GETSTATE polling thread. Clears any latched fault on startup.
        Raises ConnectionError if the gripper is unreachable.
        """
        try:
            self._sock = self._make_socket(self.ip, self.port, self.timeout)
        except (socket.error, OSError) as e:
            self._sock = None
            raise ConnectionError(
                f"WSG32: Cannot reach {self.ip}:{self.port} — {e}\n"
                f"Check: (1) gripper powered, (2) alias IP 192.168.1.100 up, "
                f"(3) cable to switch."
            )

        # Second connection for state polling — gripper accepts multiple clients
        try:
            self._state_sock = self._make_socket(self.ip, self.port, self.timeout)
        except (socket.error, OSError):
            # State feedback unavailable — not fatal, fall back to commanded pos
            self._state_sock = None

        # Clear any latched fast-stop from a previous crash
        self.ack_fault()

        # Start background polling thread
        if self._state_sock is not None:
            self._polling = True
            self._poll_thread = threading.Thread(
                target=self._poll_loop,
                daemon=True,
                name=f"wsg32_poll_{self.ip}"
            )
            self._poll_thread.start()

    def disconnect(self) -> None:
        """Stop polling thread and gracefully close both TCP sessions."""
        # Stop the background thread first
        self._polling = False
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None

        # Close feedback socket
        if self._state_sock is not None:
            try:
                self._state_sock.close()
            except Exception:
                pass
            self._state_sock = None

        # Close command socket
        if self._sock is not None:
            try:
                self._send_raw("BYE()\n")
            except Exception:
                pass
            self._sock.close()
            self._sock = None

    def is_connected(self) -> bool:
        return self._sock is not None

    # ──────────────────────────────────────────────────────────────────────
    # State feedback (background thread)
    # ──────────────────────────────────────────────────────────────────────

    @property
    def state(self) -> GripperState:
        """
        Thread-safe read of the latest measured gripper state.
        Updated at poll_hz by the background thread.
        Check state.is_fresh() to detect polling failure.
        """
        with self._state_lock:
            # Return a copy so the caller can't mutate shared state
            s = self._state
            return GripperState(
                pos_mm=s.pos_mm,
                speed_mms=s.speed_mms,
                force_n=s.force_n,
                state_str=s.state_str,
                timestamp=s.timestamp,
            )

    def _poll_loop(self) -> None:
        """
        Background thread: sends GETSTATE() on the dedicated feedback socket
        and parses the response into self._state.

        GCL GETSTATE response format (text interface):
            POS <pos_mm> SPEED <speed_mm_s> FORCE <force_n> STATE <state_name>\\n

        Example:
            POS 27.50 SPEED 0.00 FORCE 0.12 STATE IDLE\\n
        """
        while self._polling:
            loop_start = time()
            try:
                # Send query
                self._state_sock.sendall(b"GETSTATE()\n")

                # Read one response line
                buf = b""
                deadline = time() + self.timeout
                while time() < deadline:
                    chunk = self._state_sock.recv(256)
                    if not chunk:
                        raise ConnectionError("feedback socket closed")
                    buf += chunk
                    if b"\n" in buf:
                        break

                line = buf.split(b"\n")[0].decode("ascii", errors="replace").strip()
                self._parse_state_line(line)

            except Exception:
                # Socket error or parse failure — stop polling silently.
                # The ROS node will detect staleness via state.is_fresh().
                self._polling = False
                break

            # Sleep for the remainder of the poll interval
            elapsed = time() - loop_start
            sleep_time = self._poll_interval - elapsed
            if sleep_time > 0:
                sleep(sleep_time)

    def _parse_state_line(self, line: str) -> None:
        """
        Parse a GETSTATE response line and update self._state.

        Expected format:  POS <f> SPEED <f> FORCE <f> STATE <word>
        Tolerant of extra whitespace and partial responses.
        """
        try:
            tokens = line.split()
            # Build a key→value dict from adjacent token pairs
            kv: dict[str, str] = {}
            for i in range(0, len(tokens) - 1, 2):
                kv[tokens[i].upper()] = tokens[i + 1]

            with self._state_lock:
                if "POS"   in kv: self._state.pos_mm    = float(kv["POS"])
                if "SPEED" in kv: self._state.speed_mms = float(kv["SPEED"])
                if "FORCE" in kv: self._state.force_n   = float(kv["FORCE"])
                if "STATE" in kv: self._state.state_str = kv["STATE"]
                self._state.timestamp = time()
        except (ValueError, IndexError):
            # Malformed line — skip silently, keep previous state
            pass

    # ──────────────────────────────────────────────────────────────────────
    # Low-level send / receive
    # ──────────────────────────────────────────────────────────────────────

    def _send_raw(self, cmd: str) -> None:
        """Send a GCL command string. Thread-safe."""
        with self._lock:
            self._sock.sendall(cmd.encode("ascii"))

    def _recv_line(self) -> str:
        """
        Read bytes until \\n.  Returns the line as a decoded string.
        Simple and correct for GCL — every response is one line.
        """
        buf = b""
        while True:
            chunk = self._sock.recv(256)
            if not chunk:
                raise ConnectionError("WSG32: socket closed by gripper")
            buf += chunk
            if b"\n" in buf:
                line = buf.split(b"\n")[0]
                return line.decode("ascii", errors="replace").strip()

    def _send_and_wait(self, cmd: str, expected_prefix: str) -> bool:
        """
        Send command, block until a line starting with expected_prefix arrives.
        Returns True on success, False on ERR response.
        Raises TimeoutError if nothing arrives within self.timeout seconds.
        """
        self._send_raw(cmd)
        deadline = time() + self.timeout
        while time() < deadline:
            line = self._recv_line()
            if line.startswith(expected_prefix):
                return True
            if line.startswith("ERR"):
                return False
        raise TimeoutError(
            f"WSG32: timed out waiting for '{expected_prefix}' "
            f"after sending: {cmd.strip()!r}"
        )

    # ──────────────────────────────────────────────────────────────────────
    # GCL Commands
    # ──────────────────────────────────────────────────────────────────────

    def ack_fault(self) -> bool:
        """
        Acknowledge / clear a latched fast-stop or fault.
        Always call this on connect, and after any E-stop event.
        GCL: FSACK()  →  response: ACK FSACK
        """
        return self._send_and_wait("FSACK()\n", "ACK FSACK")

    def home(self) -> bool:
        """
        Run homing sequence: gripper opens fully to find the reference position.
        MUST be called at least once after power-on before any MOVE command.
        Blocks until homing completes (~3–5 seconds).
        GCL: HOME()  →  response: FIN HOME
        """
        return self._send_and_wait("HOME()\n", "FIN HOME")

    def move(self, position_mm: float) -> bool:
        """
        Move fingers to position_mm. Blocks until motion completes.
        position_mm: 0.0 = fully closed, MAX_POS_MM = fully open.

        Use this for scripted sequences. For live teleoperation, use
        move_nonblocking() instead.

        GCL: MOVE(<pos>)  →  response: FIN MOVE
        """
        position_mm = self._clamp(position_mm)
        cmd = f"MOVE({position_mm:.2f})\n"
        return self._send_and_wait(cmd, "FIN MOVE")

    def move_nonblocking(self, position_mm: float, speed_mm_s: float = 400.0) -> None:
        """
        Send a MOVE command and return immediately WITHOUT waiting for FIN MOVE.
        Uses STOP() only on direction reversal to allow mid-motion retargeting.
        speed_mm_s: jaw speed in mm/s (WSG32 max ~400mm/s)
        """
        position_mm = self._clamp(position_mm)


        # STOP only on direction reversal — prevents queuing in wrong direction
        if self._last_cmd_pos is not None and self._last_gripper_dir is not None:
            new_dir = position_mm - self._last_cmd_pos
            if new_dir != 0 and self._last_gripper_dir != 0:
                if (new_dir > 0) != (self._last_gripper_dir > 0):
                    self._send_raw("STOP()\n")
            if new_dir != 0:
                self._last_gripper_dir = new_dir
        else:
            if self._last_cmd_pos is not None:
                d = position_mm - self._last_cmd_pos
                if d != 0:
                    self._last_gripper_dir = d

        self._last_cmd_pos = position_mm
        self._send_raw(f"MOVE({position_mm:.1f},{speed_mm_s:.0f})\n")

    def release(self, open_mm: float = 10.0) -> bool:
        """
        Open fingers by open_mm to release a grasped object.
        GCL: RELEASE(<mm>)  →  response: FIN RELEASE
        """
        return self._send_and_wait(f"RELEASE({open_mm:.1f})\n", "FIN RELEASE")

    def stop(self) -> None:
        """
        Immediately stop all finger motion.
        GCL: STOP()  →  response: ACK STOP
        """
        self._send_raw("STOP()\n")

    # ──────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────

    def _clamp(self, position_mm: float) -> float:
        return max(self.MIN_POS_MM, min(self.MAX_POS_MM, position_mm))

    def normalize_to_pos(self, value_0_to_1: float) -> float:
        """
        Convert a normalized [0.0, 1.0] leader value to gripper mm.
        0.0 → fully closed (0 mm)
        1.0 → fully open (MAX_POS_MM)
        Useful if your GELLO outputs a normalized joint angle.
        """
        return value_0_to_1 * self.MAX_POS_MM