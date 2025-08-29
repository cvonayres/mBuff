#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8MultiArray

try:
    from sense_hat import SenseHat  # type: ignore
except Exception:
    SenseHat = None  # type: ignore

from .faces import normal_face, blink_face, wink_face
from .image_handler import parse_led_data, display_pixels


class MSenseHatNode(Node):
    def __init__(self) -> None:
        super().__init__('msense_hat')
        self.get_logger().info('Initialising Sense HAT node')

        # Single Sense HAT instance, rotated once
        self.sense = None
        if SenseHat is not None:
            try:
                self.sense = SenseHat()
                self.sense.set_rotation(180)
                self.get_logger().info('Sense HAT initialised and rotated 180°')
            except Exception as exc:
                self.get_logger().warning(f'Failed to initialise Sense HAT: {exc}. Using simulation.')
        else:
            self.get_logger().warning('sense_hat module not available. Using simulation.')

        # State
        self.showing_image = False
        self.blink_timer = None
        self.blink_end_timer = None
        self.wink_end_timer = None
        self.image_end_timer = None

        # Show startup face
        self.get_logger().info('Displaying startup face')
        self.display_face(normal_face())

        # Subscriber
        topic = "/mbuff/sensehat/leds"
        qos = qos_profile_sensor_data
        self.get_logger().info(f"Subscribing to '{topic}' with sensor-data QoS")
        self.led_subscription = self.create_subscription(
            UInt8MultiArray,
            topic,
            self.led_frame_callback,
            qos,
        )

        # Joystick & blink
        self.get_logger().info('Starting joystick poll @ 10 Hz')
        self.joystick_timer = self.create_timer(0.1, self.check_joystick)
        self.schedule_blink()

    # ----------------- helpers -----------------
    def display_face(self, pixels) -> None:
        """Render 8x8 (64) RGB tuples to HW if present, else simulation."""
        if self.sense is not None:
            try:
                self.sense.set_pixels(pixels)
                self.get_logger().debug('Rendered frame to hardware Sense HAT')
                return
            except Exception as exc:
                self.get_logger().warning(f'set_pixels failed: {exc}. Falling back to simulation.')

        self.get_logger().debug('Rendered frame to simulation (no hardware)')
        display_pixels(pixels)

    def cancel_timer(self, name: str) -> None:
        t = getattr(self, name)
        if t is not None:
            try:
                t.cancel()
            except Exception:
                pass
            setattr(self, name, None)
            self.get_logger().debug(f'Cancelled timer: {name}')

    def schedule_blink(self) -> None:
        if self.showing_image:
            self.get_logger().debug('Blink not scheduled: currently showing image')
            return
        self.cancel_timer('blink_timer')
        delay = random.uniform(4.0, 8.0)
        self.get_logger().debug(f'Scheduling blink in {delay:.2f}s')
        self.blink_timer = self.create_timer(delay, self.do_blink)

    # ----------------- blink -----------------
    def do_blink(self) -> None:
        if self.showing_image:
            self.get_logger().debug('Blink skipped: currently showing image')
            return
        self.get_logger().debug('Blink: show blink face')
        self.display_face(blink_face())
        self.cancel_timer('blink_timer')
        self.blink_end_timer = self.create_timer(0.3, self.end_blink)

    def end_blink(self) -> None:
        self.cancel_timer('blink_end_timer')
        if not self.showing_image:
            self.get_logger().debug('Blink ended: revert to normal, reschedule blink')
            self.display_face(normal_face())
            self.schedule_blink()
        else:
            self.get_logger().debug('Blink end suppressed: image active')

    # ----------------- wink -----------------
    def check_joystick(self) -> None:
        if self.sense is None:
            return
        if self.showing_image:
            return
        events = self.sense.stick.get_events()
        if events:
            self.get_logger().debug(f'Joystick events: {len(events)}')
        for event in events:
            if event.action == 'pressed':
                self.get_logger().info('Joystick press → wink')
                self.cancel_timer('blink_timer')
                self.cancel_timer('blink_end_timer')
                self.display_face(wink_face())
                self.cancel_timer('wink_end_timer')
                self.wink_end_timer = self.create_timer(0.5, self.end_wink)
                break

    def end_wink(self) -> None:
        self.cancel_timer('wink_end_timer')
        if not self.showing_image:
            self.get_logger().info('Wink ended → back to normal')
            self.display_face(normal_face())
            self.schedule_blink()
        else:
            self.get_logger().debug('Wink end suppressed: image active')

    # ----------------- image -----------------
    def led_frame_callback(self, msg: UInt8MultiArray) -> None:
        byte_count = len(msg.data)
        self.get_logger().info(f"LED frame received ({byte_count} bytes)")
        if byte_count == 0:
            self.get_logger().warning('Empty LED frame, ignoring')
            return

        try:
            pixels = parse_led_data(list(msg.data))
        except Exception as exc:
            self.get_logger().error(f'Failed to parse LED data: {exc}')
            return

        if len(pixels) != 64:
            self.get_logger().warning(f'Parsed pixel count != 64 ({len(pixels)}), proceeding anyway')

        self.get_logger().info('Displaying incoming image and pausing blink/wink for 15s')
        self.showing_image = True
        self.cancel_timer('blink_timer')
        self.cancel_timer('blink_end_timer')
        self.cancel_timer('wink_end_timer')
        self.cancel_timer('image_end_timer')

        # Where are we writing?
        if self.sense is not None:
            self.get_logger().debug('Image path: hardware Sense HAT')
        else:
            self.get_logger().debug('Image path: simulation (no hardware)')

        self.display_face(pixels)
        self.image_end_timer = self.create_timer(15.0, self.end_image_display)

    def end_image_display(self) -> None:
        self.cancel_timer('image_end_timer')
        self.showing_image = False
        self.get_logger().info('Image period ended → back to normal face, resume blinking')
        self.display_face(normal_face())
        self.schedule_blink()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MSenseHatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
