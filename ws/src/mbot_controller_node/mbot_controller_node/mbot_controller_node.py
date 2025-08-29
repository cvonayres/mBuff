"""
ROS2 node to control the mBot via serial, with persistent connection + detailed logging.

Env:
  MBOT_SERIAL_PORT  (default: /dev/ttyUSB0)
  MBOT_BAUDRATE     (default: 9600)
  MBOT_MOVE_TOPIC   (default: /mbuff/move)
  MBOT_LOG_LEVEL    (DEBUG|INFO|WARN|ERROR|FATAL, default: INFO)
  MBOT_READBACK_MS  (default: 0; if >0, tries to read a line from serial)
  MBOT_DTR_RESET    (default: 0; if 1, allow DTR high which resets Arduino)
"""

import os
import time
import serial  # type: ignore
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String

ALLOWED_COMMANDS = {
    'FORWARD','BACKWARD','LEFT','RIGHT','STOP',
    'SPIN_LEFT','SPIN_RIGHT','SPIN_BACK_LEFT','SPIN_BACK_RIGHT',
    'SPEEDUP','SPEEDDOWN',
    'DANCE1','DANCE2','DANCE3','DANCE4',
    'SONG1','SONG2','SONG3','SONG4',
    'SAYING1','SAYING2','SAYING3','SAYING4','SAYING5','SAYING6',
}

def _parse_log_level(val: Optional[str]) -> int:
    mapping = {
        'DEBUG': LoggingSeverity.DEBUG,
        'INFO': LoggingSeverity.INFO,
        'WARN': LoggingSeverity.WARN,
        'WARNING': LoggingSeverity.WARN,
        'ERROR': LoggingSeverity.ERROR,
        'FATAL': LoggingSeverity.FATAL,
        'CRITICAL': LoggingSeverity.FATAL,
    }
    if not val:
        return LoggingSeverity.INFO
    return mapping.get(val.strip().upper(), LoggingSeverity.INFO)

class MBotControllerNode(Node):
    """ROS2 node that proxies movement commands to an mBot over serial (persistent port)."""

    def __init__(self) -> None:
        super().__init__('mbot_controller')

        # Log level
        env_level = os.getenv('MBOT_LOG_LEVEL')
        self.get_logger().set_level(_parse_log_level(env_level))

        # Config
        self.serial_port: str = os.getenv('MBOT_SERIAL_PORT', '/dev/ttyUSB0')
        self.baud_rate: int = int(os.getenv('MBOT_BAUDRATE', '9600'))
        self.move_topic: str = os.getenv('MBOT_MOVE_TOPIC', '/mbuff/move')
        self.readback_ms: int = int(os.getenv('MBOT_READBACK_MS', '0'))
        self.serial_timeout: float = max(0.001, self.readback_ms / 1000.0)
        self.serial_init_delay: float = 1.5   # allow MCU to settle once after first open
        self.allow_dtr_reset: bool = os.getenv('MBOT_DTR_RESET', '0').strip() in ('1', 'true', 'TRUE')

        self.get_logger().info(
            "mBot controller init: "
            f"serial_port={self.serial_port}, baud={self.baud_rate}, "
            f"move_topic={self.move_topic}, readback_ms={self.readback_ms}, "
            f"dtr_reset={'ON' if self.allow_dtr_reset else 'OFF'}, "
            f"log_level_env={env_level or 'INFO(default)'}"
        )

        # Serial handle
        self.ser: Optional[serial.Serial] = None
        self._first_open_done = False
        self._last_open_attempt = 0.0
        self._reopen_backoff_s = 2.0

        # Subscription
        self.subscription = self.create_subscription(String, self.move_topic, self._move_callback, 10)
        tname = getattr(self.subscription, 'topic_name', self.move_topic)
        self.get_logger().info(f"Subscribed to topic: {tname}")

        # Heartbeat + periodic serial health check
        self._hb_timer = self.create_timer(10.0, self._heartbeat)
        self._serial_timer = self.create_timer(1.0, self._ensure_serial_periodic)

        # Try initial open immediately
        self._open_serial(initial=True)

    # ---------- Serial management ----------

    def _open_serial(self, initial: bool = False) -> None:
        now = time.time()
        # Throttle repeated attempts
        if (now - self._last_open_attempt) < 0.5:
            return
        self._last_open_attempt = now

        try:
            self.get_logger().debug(f"[SER] Opening {self.serial_port} @ {self.baud_rate} (timeout={self.serial_timeout}s)")
            ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=self.serial_timeout,
                rtscts=False,
                dsrdtr=False,
                write_timeout=1.0,
            )
            # Kill DTR/RTS unless caller explicitly wants auto-reset behaviour
            if not self.allow_dtr_reset:
                try:
                    ser.setDTR(False)
                    ser.setRTS(False)
                except Exception:
                    pass

            self.ser = ser

            # Only do the long settle once (handle Arduino auto-reset on first open)
            if not self._first_open_done:
                time.sleep(self.serial_init_delay)
                self._first_open_done = True

            self.get_logger().info(f"[SER] Opened {self.serial_port} (persistent).")
        except Exception as e:
            self.ser = None
            self.get_logger().warn(f"[SER] Open failed: {e} (will retry)")

    def _ensure_serial(self) -> bool:
        if self.ser and self.ser.is_open:
            return True
        self._open_serial()
        return bool(self.ser and self.ser.is_open)

    def _ensure_serial_periodic(self) -> None:
        # Called by timer to reconnect if cable re-plugged etc.
        if not self._ensure_serial():
            # exponential-ish backoff cap
            self._reopen_backoff_s = min(self._reopen_backoff_s * 1.5, 10.0)
        else:
            self._reopen_backoff_s = 2.0

    def _close_serial(self) -> None:
        if self.ser:
            try:
                self.ser.close()
                self.get_logger().info("[SER] Closed serial port.")
            except Exception as e:
                self.get_logger().warn(f"[SER] Close error: {e}")
            finally:
                self.ser = None

    # ---------- ROS callbacks ----------

    def _heartbeat(self) -> None:
        self.get_logger().debug("Heartbeat: node alive")

    def _move_callback(self, msg: String) -> None:
        recv_ts = time.time()
        payload = msg.data if isinstance(msg.data, str) else str(msg.data)

        topic = getattr(self.subscription, 'topic_name', self.move_topic)
        self.get_logger().info(f"[RX] {recv_ts:.3f} topic={topic} msg='{payload}' len={len(payload)}")

        command = payload.strip()
        if not command:
            self.get_logger().warning("[DROP] Empty command payload")
            return

        serial_cmd = command.upper()
        if serial_cmd not in ALLOWED_COMMANDS:
            self.get_logger().warning(f"[DROP] Unsupported command '{command}'. Allowed: {', '.join(sorted(ALLOWED_COMMANDS))}")
            return

        if not self._ensure_serial():
            self.get_logger().error("[ERROR] Serial not available; dropping command")
            return

        try:
            out = (serial_cmd + '\n').encode('utf-8')
            n = self.ser.write(out)  # type: ignore[union-attr]
            self.ser.flush()         # type: ignore[union-attr]
            self.get_logger().info(f"[TX] '{serial_cmd}\\n' ({n} bytes) → {self.serial_port}")

            if self.readback_ms > 0:
                try:
                    line = self.ser.readline()  # type: ignore[union-attr]
                    if line:
                        self.get_logger().info(f"[RX-SERIAL] {line.decode(errors='replace').rstrip()}")
                    else:
                        self.get_logger().debug("[RX-SERIAL] No readback")
                except Exception as re:
                    self.get_logger().warning(f"[RX-SERIAL] Readback error: {re}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Write failed: {e}")
            self._close_serial()  # force reopen next time

    # ---------- Lifecycle ----------

    def destroy_node(self):
        try:
            self._close_serial()
        finally:
            super().destroy_node()

def main() -> None:
    rclpy.init()
    node = MBotControllerNode()
    node.get_logger().info("mBot controller node started; spinning… (Ctrl+C to stop)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received; shutting down.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception in spin: {e}")
    finally:
        node.get_logger().info("Destroying node…")
        node.destroy_node()
        rclpy.shutdown()
