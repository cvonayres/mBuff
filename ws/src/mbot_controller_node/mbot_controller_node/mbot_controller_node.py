"""ROS2 node to control the mBot via serial.

This node subscribes to a movement command topic published by the web
bridge and relays the commands over a serial connection to the mBot.

It expects the commands to be simple strings (e.g. ``forward``,
``backward``, ``left``, ``right``, ``stop``) which are converted to
uppercase before being transmitted. The serial port and baud rate can
be configured via the ``MBOT_SERIAL_PORT`` and ``MBOT_BAUDRATE``
environment variables respectively. If unset, they default to
``/dev/ttyUSB0`` and ``9600``.  Adjust these values as appropriate
for your hardware.
"""

import os
import time
import serial  # type: ignore

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Define the set of valid high-level commands that can be sent to the mBot.
#
# The web bridge or other upstream client should publish one of these on
# `/mbuff/move`.  Commands are case-insensitive on input but will be
# transmitted in uppercase.  Any unrecognised command will be logged
# and dropped.  Feel free to extend this set as you add new behaviours
# to the mBot firmware.
ALLOWED_COMMANDS = {
    # Basic movement
    'FORWARD',       # Move the robot forward
    'BACKWARD',      # Move the robot backwards
    'LEFT',          # Move the robot left
    'RIGHT',         # Move the robot right
    'STOP',          # Stop all movement

    # Spinning (corner buttons)
    'SPIN_LEFT',       # Spin on the spot to the left
    'SPIN_RIGHT',      # Spin on the spot to the right
    'SPIN_BACK_LEFT',  # Spin backwards to the left
    'SPIN_BACK_RIGHT', # Spin backwards to the right

    # Speed control
    'SPEEDUP',    # Increase movement speed
    'SPEEDDOWN',  # Decrease movement speed

    # Dance routines (IDs 1–4)
    'DANCE1', 'DANCE2', 'DANCE3', 'DANCE4',

    # Songs (IDs 1–4)
    'SONG1', 'SONG2', 'SONG3', 'SONG4',

    # Sayings (IDs 1–6)
    'SAYING1', 'SAYING2', 'SAYING3', 'SAYING4', 'SAYING5', 'SAYING6',
}


class MBotControllerNode(Node):
    """ROS2 node that proxies movement commands to an mBot over serial."""

    def __init__(self) -> None:
        super().__init__('mbot_controller')
        # Serial connection parameters
        self.serial_port: str = os.getenv('MBOT_SERIAL_PORT', '/dev/ttyUSB0')
        self.baud_rate: int = int(os.getenv('MBOT_BAUDRATE', '9600'))

        # Log our configuration
        self.get_logger().info(
            f'mBot controller initialised; serial port={self.serial_port}, baud={self.baud_rate}'
        )

        # Create a subscription to the movement command topic. Use the same topic
        # that the web bridge publishes to. See web_bridge_node/web_bridge.py for
        # details. We use QoS depth 10 (reliable, keep last) to ensure we
        # receive commands even under bursty conditions.
        self.subscription = self.create_subscription(
            String,
            '/mbuff/move',
            self._move_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

    def _move_callback(self, msg: String) -> None:
        """Handle incoming movement commands and forward them to the mBot.

        The incoming message data is expected to be a simple direction
        string. The command is normalised to uppercase before sending to the
        serial device. Any errors communicating with the serial port are
        logged.
        """
        command = msg.data.strip()
        if not command:
            # Ignore empty messages silently.
            return
        # Normalise to uppercase for comparison and transmission
        serial_cmd = command.upper()
        if serial_cmd not in ALLOWED_COMMANDS:
            # Warn about unsupported commands but do not send them
            self.get_logger().warning(
                f'Received unsupported command "{command}"; supported commands are: '
                f"{', '.join(sorted(ALLOWED_COMMANDS))}"
            )
            return
        self.get_logger().info(
            f'Received move command: "{command}" → sending "{serial_cmd}" to mBot'
        )
        try:
            with serial.Serial(self.serial_port, self.baud_rate, timeout=2) as ser:
                # Allow the serial connection time to stabilise.  This delay
                # was chosen to balance responsiveness with reliability; adjust
                # ``SERIAL_INIT_DELAY`` below if necessary.
                SERIAL_INIT_DELAY = 1.0
                time.sleep(SERIAL_INIT_DELAY)
                ser.write((serial_cmd + '\n').encode('utf-8'))
                ser.flush()
            self.get_logger().info('Command sent to mBot successfully')
        except Exception as exc:
            self.get_logger().error(f'Failed to send command to mBot: {exc}')

def main() -> None:
    """Entry point for the mBot controller node."""
    rclpy.init()
    node = MBotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
