import rclpy
from rclpy.node import Node

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge')
        self.timer = self.create_timer(2.0, self._tick_callback)
        self.get_logger().info('web_bridge node up (stub)')

    def _tick_callback(self):
        self.get_logger().info('web_bridge heartbeat…')

def main():
    rclpy.init()
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
