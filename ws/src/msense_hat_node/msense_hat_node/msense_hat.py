import os, sys
import rclpy
from rclpy.node import Node

class MSenseHatNode(Node):
    def __init__(self):
        super().__init__('msense_hat')
        debug = ('-d' in sys.argv) or ('--debug' in sys.argv) or (os.getenv('DEBUG') == '1')
        if debug:
            self._timer = self.create_timer(2.0, self._tick_callback)
        self.get_logger().info('msense_hat node up (stub)')

    def _tick_callback(self):
        self.get_logger().debug('msense_hat heartbeat…')

def main():
    rclpy.init()
    node = MSenseHatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
