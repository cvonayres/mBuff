from sense_hat import SenseHat
from time import sleep

sense = SenseHat()

# Start with a sensible guess; Pi heat skews temp high.
TEMP_OFFSET_C = -9.0   # tweak with the joystick (up/down)

def fill(color):
    """Fill the 8x8 with a single colour."""
    sense.set_pixels([color] * 64)

def pick_icon_colour(pressure_hpa: float, humidity_pct: float):
    """
    Very simple 'weather':
    - High pressure & not humid => sunny (yellow)
    - Very humid or low pressure => rainy (blue)
    - Otherwise => cloudy (light grey)
    """
    if pressure_hpa > 1018 and humidity_pct < 60:
        return (255, 200, 0)      # sunny
    elif humidity_pct > 80 or pressure_hpa < 1005:
        return (0, 120, 255)      # rainy
    else:
        return (220, 220, 220)    # cloudy

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

try:
    # Optional hello
    sense.show_message("Weather Face", scroll_speed=0.05, text_colour=(255,255,255), back_colour=(0,0,0))

    offset = TEMP_OFFSET_C

    while True:
        # Read sensors
        t = sense.get_temperature_from_pressure() + offset
        h = sense.get_humidity()
        p = sense.get_pressure()

        # Choose background/icon colour
        fill(pick_icon_colour(p, h))

        # Handle joystick offset tweaks (non-blocking)
        for e in sense.stick.get_events():
            if e.action == "pressed":
                if e.direction == "up":
                    offset = clamp(offset + 0.5, -20.0, 20.0)
                    sense.show_message(f"Off {offset:+.1f}C", scroll_speed=0.05, text_colour=(255,255,255), back_colour=(0,0,0))
                elif e.direction == "down":
                    offset = clamp(offset - 0.5, -20.0, 20.0)
                    sense.show_message(f"Off {offset:+.1f}C", scroll_speed=0.05, text_colour=(255,255,255), back_colour=(0,0,0))
                elif e.direction == "middle":
                    sense.show_message(f"Offset {offset:+.1f}C", scroll_speed=0.05, text_colour=(255,255,255), back_colour=(0,0,0))

        # Show readings (white text on black so it's readable)
        msg = f"{t:.1f}C {h:.0f}% {p:.0f}hPa"
        sense.show_message(msg, scroll_speed=0.05, text_colour=(255,255,255), back_colour=(0,0,0))

        # Small pause before next cycle
        sleep(0.3)

except KeyboardInterrupt:
    sense.clear()
