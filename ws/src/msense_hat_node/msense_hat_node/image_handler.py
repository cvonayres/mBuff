"""
Utilities for converting and displaying LED frames on the Sense HAT.
This module is separated from the ROS2 node so that other components
can use it independently.
"""
from typing import List, Tuple, Optional
import time

# Attempt to import SenseHat hardware; if unavailable, use None.
try:
    from sense_hat import SenseHat  # type: ignore
    _sense: Optional[SenseHat] = SenseHat()
except Exception:
    _sense = None


Pixel = Tuple[int, int, int]


def clamp_pixels(pixels: List[Pixel]) -> List[Pixel]:
    """Clamp pixel values to the 0-255 range."""
    return [(
        max(0, min(255, r)),
        max(0, min(255, g)),
        max(0, min(255, b)),
    ) for (r, g, b) in pixels]


def display_pixels(pixels: List[Pixel]) -> None:
    """
    Display a sequence of 64 RGB pixels on the Sense HAT if available.
    If the hardware is not present, a short preview of the frame will
    be printed instead. The input list should contain exactly 64
    3-tuples of (r, g, b).
    """
    if len(pixels) != 64:
        raise ValueError(f"Expected 64 pixels, got {len(pixels)}")
    clamped = clamp_pixels(pixels)
    if _sense:
        try:
            _sense.set_pixels(clamped)  # row-major 8x8
        except Exception:
            # On error, clear the display rather than crashing
            try:
                _sense.clear()
            except Exception:
                pass
    else:
        # Simulation: log first few pixels
        print(f"(sim) LED frame received (showing 8/64): {clamped[:8]}")


def parse_led_data(data: List[int]) -> List[Pixel]:
    """
    Convert a flat list of 192 integers (RGB triplets) into a list of
    64 RGB tuples. Values outside the 0-255 range are clamped.
    """
    if len(data) != 64 * 3:
        raise ValueError(f"Bad frame: expected 192 values, got {len(data)}")
    # Convert flat [r,g,b,r,g,b,...] -> [(r,g,b)] x 64
    pixels: List[Pixel] = [
        (data[i], data[i + 1], data[i + 2]) for i in range(0, len(data), 3)
    ]
    return clamp_pixels(pixels)


def show_for_duration(pixels: List[Pixel], duration: float = 15.0) -> None:
    """
    Display the provided pixels for a given duration (seconds), then clear the display.
    """
    display_pixels(pixels)
    # Wait for the specified duration; during this time the ROS node may be spinning
    time.sleep(duration)
    # After the duration, if hardware is available, revert to blank (off) state
    if _sense:
        try:
            _sense.clear()
        except Exception:
            pass
