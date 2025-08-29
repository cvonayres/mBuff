# msense_hat_node.py
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    from sense_hat import SenseHat
except ImportError:
    SenseHat = None

from .faces import normal_face, blink_face, wink_face
from . import image_handler as ih

class MsenseHatNode(Node):
    def __init__(self):
        super().__init__('msense_hat_node')
        # initialise Sense HAT hardware if available
        if SenseHat is not None:
            try:
                self.sense = SenseHat()
            except Exception as exc:
                self.get_logger().warning(f"Sense HAT not detected: {exc}. Using simulation.")
                self.sense = None
        else:
            self.sense = None

        # show normal face at startup
        self.showing_image = False
        self.display_face(normal_face())

        # subscriber to LED frame
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            'led_frame',
            self.led_frame_callback,
            10)

        # timers
        self.image_timer = None
        self.blink_timer = None
        self.blink_end_timer = None
        self.wink_end_timer = None
        self.schedule_blink()

        # joystick monitoring timer
        # only meaningful when hardware present
        self.joystick_timer = self.create_timer(0.2, self.check_joystick)

    def display_pixels(self, pixels):
        """Display pixels either on real Sense HAT or via simulation."""
        if self.sense:
            try:
                self.sense.set_pixels(pixels)
            except Exception as exc:
                self.get_logger().warning(f"Failed to set pixels on Sense HAT: {exc}")
        else:
            ih.display_pixels(pixels)

    def display_face(self, face):
        """Display one of the predefined faces."""
        self.display_pixels(face)

    def led_frame_callback(self, msg: UInt8MultiArray):
        """Handle incoming LED frame messages."""
        pixels = ih.parse_led_data(msg.data)
        # cancel any blink or wink in progress
        self.showing_image = True
        self.display_pixels(pixels)
        # cancel previous image timer
        if self.image_timer:
            try:
                self.image_timer.cancel()
            except Exception:
                pass
        # schedule revert to normal face after 15 seconds
        self.image_timer = self.create_timer(15.0, self.end_image_display)

    def end_image_display(self):
        """End display of an image and revert to normal face."""
        if self.image_timer:
            try:
                self.image_timer.cancel()
            except Exception:
                pass
            self.image_timer = None
        self.showing_image = False
        self.display_face(normal_face())

    def schedule_blink(self):
        """Schedule the next blink at a random interval."""
        interval = random.uniform(4.0, 8.0)
        if self.blink_timer:
            try:
                self.blink_timer.cancel()
            except Exception:
                pass
        self.blink_timer = self.create_timer(interval, self.do_blink)

    def do_blink(self):
        """Perform a blink and schedule the next blink."""
        # schedule next blink
        self.schedule_blink()
        if self.showing_image:
            return
        # show blink face then revert quickly
        self.display_face(blink_face())
        # schedule end of blink
        if self.blink_end_timer:
            try:
                self.blink_end_timer.cancel()
            except Exception:
                pass
        self.blink_end_timer = self.create_timer(0.3, self.end_blink)

    def end_blink(self):
        """End blink and revert to normal face."""
        if self.blink_end_timer:
            try:
                self.blink_end_timer.cancel()
            except Exception:
                pass
            self.blink_end_timer = None
        if not self.showing_image:
            self.display_face(normal_face())

    def check_joystick(self):
        """Check the joystick and trigger a wink on press."""
        if not self.sense:
            return
        events = self.sense.stick.get_events()
        for event in events:
            if event.action == 'pressed':
                # trigger wink if not currently showing image
                if not self.showing_image:
                    self.display_face(wink_face())
                    if self.wink_end_timer:
                        try:
                            self.wink_end_timer.cancel()
                        except Exception:
                            pass
                    self.wink_end_timer = self.create_timer(0.5, self.end_wink)

    def end_wink(self):
        """End wink and revert to normal."""
        if self.wink_end_timer:
            try:
                self.wink_end_timer.cancel()
            except Exception:
                pass
            self.wink_end_timer = None
        if not self.showing_image:
            self.display_face(normal_face())

def main(args=None):
    rclpy.init(args=args)
    node = MsenseHatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
