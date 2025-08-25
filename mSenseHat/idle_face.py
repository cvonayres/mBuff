from sense_hat import SenseHat
from time import sleep, time
import random

sense = SenseHat()

# Colours
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED   = (255, 0, 0)
PINK  = (255, 163, 177)

def show(face_pixels):
    """Validate and display a 64-pixel frame."""
    if not isinstance(face_pixels, list) or len(face_pixels) != 64:
        raise ValueError(f"Face must be a list[64], got {type(face_pixels)} len={len(face_pixels) if isinstance(face_pixels, list) else 'N/A'}")
    sense.set_pixels(face_pixels)

# Faces
def normal_face():
    face = [BLACK] * 64
    # eyes
    face[2*8 + 2] = WHITE
    face[2*8 + 5] = WHITE
    # curved smile (your layout)
    face[5*8 + 2] = RED
    face[6*8 + 3] = RED
    face[6*8 + 4] = RED
    face[5*8 + 5] = RED
    return face

def blink_face():
    face = [BLACK] * 64
    # keep the same mouth
    face[5*8 + 2] = RED
    face[6*8 + 3] = RED
    face[6*8 + 4] = RED
    face[5*8 + 5] = RED
    return face

def wink_face():
    face = [BLACK] * 64
    # a cute wink + blush using your PINK pixels
    face[1*8 + 1] = WHITE   # eyebrow sparkle
    face[2*8 + 2] = WHITE   # left eye open
    face[3*8 + 1] = WHITE   # accent
    # mouth (two corners)
    face[4*8 + 2] = RED
    face[4*8 + 5] = RED
    # middle mouth arc
    for x in range(2, 6):
        face[5*8 + x] = RED
    # blush
    face[6*8 + 3] = PINK
    face[6*8 + 4] = PINK
    face[7*8 + 3] = PINK
    face[7*8 + 4] = PINK
    return face

try:
    show(normal_face())
    last_change = time()

    while True:
        # random blink
        if time() - last_change > random.uniform(4, 8):
            show(blink_face())
            sleep(0.3)
            show(normal_face())
            last_change = time()

        # joystick wink
        for e in sense.stick.get_events():
            if e.action == "pressed" and e.direction == "middle":
                show(wink_face())
                sleep(0.5)
                show(normal_face())

        sleep(0.05)

except KeyboardInterrupt:
    sense.clear()
