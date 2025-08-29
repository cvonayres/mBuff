import os, sys
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

# Try to load Sense HAT hardware; fall back to a stub logger
try:
    from sense_hat import SenseHat  # type: ignore
    _sense: Optional[SenseHat] = SenseHat()
except Exception:
    _sense = None

DEFAULT_TOPIC = os.getenv("SENSEHAT_TOPIC", "/mbuff/sensehat/leds")


class MSenseHatNode(Node):
    def __init__(self):
        super().__init__('msense_hat')

        # Subscribes to 64x3 RGB (flat) LED frames
        self.sub = self.create_subscription(
            UInt8MultiArray, DEFAULT_TOPIC, self._on_leds, 10
        )

        # Optional heartbeat
        debug = ('-d' in sys.argv) or ('--debug' in sys.argv) or (os.getenv('DEBUG') == '1')
        if debug:
            self._timer = self.create_timer(2.0, self._tick_callback)

        if _sense:
            try:
                # Hard-coded rotation: 180 degrees
                _sense.set_rotation(180)
            except Exception as e:
                self.get_logger().warn(f'Failed to set rotation: {e}')
            self.get_logger().info(
                f'msense_hat node up — hardware detected (rotation=180), listening on {DEFAULT_TOPIC}'
            )
        else:
            self.get_logger().warn(
                f'msense_hat node up — Sense HAT not available, sim-only. Listening on {DEFAULT_TOPIC}'
            )

    def _tick_callback(self):
        self.get_logger().debug('msense_hat heartbeat…')

    def _on_leds(self, msg: UInt8MultiArray):
        data = list(msg.data)
        if len(data) != 64 * 3:
            self.get_logger().warn(f"Bad frame: expected 192 values, got {len(data)}")
            return

        # Convert flat [r,g,b,r,g,b,...] -> [(r,g,b)] x 64
        pixels: List[Tuple[int, int, int]] = [
            (data[i], data[i + 1], data[i + 2]) for i in range(0, 192, 3)
        ]

        # Clamp defensively (0..255), just in case
        pixels = [(
            max(0, min(255, r)),
            max(0, min(255, g)),
            max(0, min(255, b)),
        ) for (r, g, b) in pixels]

        if _sense:
            try:
                # Rotation already handled by Sense HAT firmware
                _sense.set_pixels(pixels)  # row-major 8x8
            except Exception as e:
                self.get_logger().error(f"Failed to set Sense HAT pixels: {e}")
        else:
            # Simulation: log first few pixels to avoid spam
            preview = pixels[:8]
            self.get_logger().info(f"(sim) LED frame received (showing 8/64): {preview}")

    def destroy_node(self):
        # Optional: clear LEDs on shutdown if we have hardware
        try:
            if _sense:
                _sense.clear()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MSenseHatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
