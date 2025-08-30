from typing import List
from fastapi import FastAPI, HTTPException
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension

class SenseAPI:
    def __init__(self, node, app: FastAPI):
        self.node = node
        self.pub = node.create_publisher(UInt8MultiArray, "/mbuff/sensehat/leds", 10)

        @app.post("/api/sensehat/leds")
        def set_leds(payload: dict):
            pixels: List[List[int]] = payload.get("pixels", [])
            if len(pixels) != 64:
                raise HTTPException(status_code=400, detail="pixels must be length 64 of [r,g,b]")
            flat: list[int] = []
            for i, px in enumerate(pixels):
                if not isinstance(px, list) or len(px) != 3:
                    raise HTTPException(status_code=400, detail=f"pixel {i} must be [r,g,b]")
                r,g,b = px
                for c in (r,g,b):
                    if not isinstance(c, int) or c < 0 or c > 255:
                        raise HTTPException(status_code=400, detail=f"pixel {i} out-of-range 0..255")
                flat.extend([r,g,b])
            self._publish(flat)
            return {"ok": True, "count": 64}

        @app.post("/api/sensehat/clear")
        def clear():
            self._publish([0]*192)
            return {"ok": True, "cleared": True}

    def _publish(self, flat_rgb: list[int]):
        if len(flat_rgb) != 192:
            self.node.get_logger().warn(f"Sense HAT publish dropped: need 192 bytes, got {len(flat_rgb)}")
            return
        msg = UInt8MultiArray()
        pix = MultiArrayDimension(); pix.label="pixels"; pix.size=64; pix.stride=192
        rgb = MultiArrayDimension(); rgb.label="rgb";    rgb.size=3;  rgb.stride=3
        msg.layout.dim = [pix, rgb]
        msg.data = flat_rgb
        self.pub.publish(msg)
        self.node.get_logger().info("WEB SENSEHAT: published 64x3 RGB frame")
