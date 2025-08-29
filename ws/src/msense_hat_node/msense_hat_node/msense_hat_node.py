#!/usr/bin/env python3
"""
ROS2 node to control the Sense HAT display with blinking, winking and image support.

This node displays a normal face on boot, performs random blinks, winks when the joystick
is pressed and shows images sent over the `led_frame` topic for 15 seconds. When an
image finishes, the normal blink cycle resumes.
"""

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


class MsenseHatNode(Node):
    """ROS2 node managing the Sense HAT LED display."""

    def __init__(self) -> None:
        super().__init__('msense_hat_node')
        self.get_logger().info('Initialising Sense HAT node')

        # Attempt to initialise hardware Sense HAT. If unavailable, simulation will be used.
        if SenseHat is not None:
            try:
                self.sense = SenseHat()
                # Rotate the display 180 degrees so that it appears right‑side up on the robot
                self.sense.set_rotation(180)
                self.get_logger().info('Sense HAT initialised')
            except Exception as exc:
                self.get_logger().warning(f'Failed to initialise Sense HAT: {exc}, falling back to simulation')
                self.sense = None
        else:
            self.sense = None

        # State flags and timers
        self.showing_image = False
        self.blink_timer = None
        self.blink_end_timer = None
        self.wink_end_timer = None

        # Display the normal face on startup
        self.display_face(normal_face())

        # Subscription to LED frames coming from the web bridge
        self.led_subscription = self.create_subscription(
            UInt8MultiArray,
            'led_frame',
            self.led_frame_callback,
            qos_profile_sensor_data,
        )

        # Poll the joystick at a fast rate for responsive winking
        self.joystick_timer = self.create_timer(0.1, self.check_joystick)

        # Schedule the first blink
        self.schedule_blink()

    # -------------------------------------------------------------------------
    # Helper methods
    def display_face(self, pixels):
    def display_face(self, pixels):
        """Display a list of 64 RGB tuples on the Sense HAT or print them if simulated."""
        display_pixels(pixels)


    def schedule_blink(self) -> None:
        """Schedule a random blink if not currently showing an image."""
        if self.showing_image:
            return
        # Cancel any existing blink timer
        if self.blink_timer is not None:
            try:
                self.blink_timer.cancel()
            except Exception:
                pass
            self.blink_timer = None
        # Choose random delay between 4 and 8 seconds
        delay = random.uniform(4.0, 8.0)
        self.blink_timer = self.create_timer(delay, self.do_blink)

    # -------------------------------------------------------------------------
    # Blink handling
    def do_blink(self) -> None:
        """Perform a blink and schedule its end."""
        # If an image is being shown, skip blinking
        if self.showing_image:
            return
        # Show blink face
        self.display_face(blink_face())
        # Cancel the blink timer so it doesn't fire again until rescheduled
        if self.blink_timer is not None:
            try:
                self.blink_timer.cancel()
            except Exception:
                pass
            self.blink_timer = None
        # Schedule the end of the blink after 0.3 seconds
        self.blink_end_timer = self.create_timer(0.3, self.end_blink)

    def end_blink(self) -> None:
        """Revert from blink to the normal face and reschedule the next blink."""
        # Cancel any pending blink end timer
        if self.blink_end_timer is not None:
            try:
                self.blink_end_timer.cancel()
            except Exception:
                pass
            self.blink_end_timer = None
        # Only revert if no image is being displayed
        if not self.showing_image:
            self.display_face(normal_face())
            self.schedule_blink()

    # -------------------------------------------------------------------------
    # Wink handling
    def check_joystick(self) -> None:
        """Poll the joystick and trigger a wink on press."""
        if self.sense is None or self.showing_image:
            return
        events = self.sense.stick.get_events()
        for event in events:
            if event.action == 'pressed':
                # Cancel blink timers so they don't interfere
                if self.blink_timer is not None:
                    try:
                        self.blink_timer.cancel()
                    except Exception:
                        pass
                    self.blink_timer = None
                if self.blink_end_timer is not None:
                    try:
                        self.blink_end_timer.cancel()
                    except Exception:
                        pass
                    self.blink_end_timer = None
                # Show wink face
                self.display_face(wink_face())
                # Cancel any existing wink end timer and schedule a new one
                if self.wink_end_timer is not None:
                    try:
                        self.wink_end_timer.cancel()
                    except Exception:
                        pass
                    self.wink_end_timer = None
                self.wink_end_timer = self.create_timer(0.5, self.end_wink)
                # Only handle one press per polling cycle
                break

    def end_wink(self) -> None:
        """Revert from a wink to the normal face and restart blinking."""
        if self.wink_end_timer is not None:
            try:
                self.wink_end_timer.cancel()
            except Exception:
                pass
            self.wink_end_timer = None
        # Revert only if no image is being displayed
        if not self.showing_image:
            self.display_face(normal_face())
            self.schedule_blink()

    # -------------------------------------------------------------------------
    # Image handling
    def led_frame_callback(self, msg: UInt8MultiArray) -> None:
        """Handle incoming LED frames and display them for 15 seconds."""
        if not msg.data:
            return
        # Parse the incoming data into 64 (r, g, b) tuples
        try:
            pixels = parse_led_data(list(msg.data))
        except Exception as exc:
            self.get_logger().error(f'Failed to parse LED data: {exc}')
            return
        # Cancel any active blink or wink timers and mark that we are showing an image
        self.showing_image = True
        for timer_attr in ('blink_timer', 'blink_end_timer', 'wink_end_timer'):
            timer = getattr(self, timer_attr)
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
                setattr(self, timer_attr, None)
        # Display the image
        display_pixels(pixels)
        # Schedule reversion after 15 seconds
        self.create_timer(15.0, self.end_image_display)

    def end_image_display(self) -> None:
        """Called when the image display duration ends to restore normal operation."""
        # Reset the flag and revert to the normal face
        self.showing_image = False
        self.display_face(normal_face())
        # Restart the blink cycle
        self.schedule_blink()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MsenseHatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicitly destroy the node and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()
