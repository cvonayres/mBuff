# Auto-generated faces definitions for MSenseHat
from typing import List, Tuple

# Colour constants
BLACK: Tuple[int, int, int] = (0, 0, 0)
WHITE: Tuple[int, int, int] = (255, 255, 255)
RED: Tuple[int, int, int] = (255, 0, 0)
PINK: Tuple[int, int, int] = (255, 163, 177)


def normal_face() -> List[Tuple[int, int, int]]:
    """Return a list of 64 RGB tuples for the normal (smiling) face."""
    face: List[Tuple[int, int, int]] = [BLACK] * 64
    # eyes (row 2, columns 2 and 5)
    face[2 * 8 + 2] = WHITE
    face[2 * 8 + 5] = WHITE
    # curved smile (your layout)
    face[5 * 8 + 2] = RED
    face[5 * 8 + 3] = RED
    face[5 * 8 + 4] = RED
    face[5 * 8 + 5] = RED
    face[6 * 8 + 2] = RED
    face[6 * 8 + 3] = RED
    face[6 * 8 + 4] = RED
    return face


def blink_face() -> List[Tuple[int, int, int]]:
    """Return a list of 64 RGB tuples for a blink face (both eyes closed)."""
    face = normal_face().copy()
    # draw closed eyes (rows 2-3)
    face[2 * 8 + 2] = BLACK
    face[2 * 8 + 5] = BLACK
    face[3 * 8 + 2] = BLACK
    face[3 * 8 + 5] = BLACK
    return face


def wink_face() -> List[Tuple[int, int, int]]:
    """Return a list of 64 RGB tuples for a wink face (right eye closed, blush).
    This follows the original idle_face wink_face from the mSenseHat project.
    """
    face: List[Tuple[int, int, int]] = [BLACK] * 64
    # eyebrow sparkle / accent
    face[1 * 8 + 1] = WHITE
    face[2 * 8 + 2] = WHITE  # left eye open
    face[3 * 8 + 1] = WHITE  # accent
    # mouth (two corners)
    face[4 * 8 + 2] = RED
    face[4 * 8 + 5] = RED
    # middle mouth arc
    for x in range(2, 6):
        face[5 * 8 + x] = RED
    # blush
    face[6 * 8 + 3] = PINK
    face[6 * 8 + 4] = PINK
    face[7 * 8 + 3] = PINK
    face[7 * 8 + 4] = PINK
    return face
