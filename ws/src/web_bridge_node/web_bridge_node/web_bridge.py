# web_bridge_node/web_bridge.py
import os, sys, threading
from enum import Enum
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray, MultiArrayDimension

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn


class Move(str, Enum):
    FORWARD  = "forward"
    BACKWARD = "backward"
    LEFT     = "left"
    RIGHT    = "right"
    STOP     = "stop"


class Command(BaseModel):
    action: Move
    speed: Optional[int] = None
    note: Optional[str] = None


class SenseHatPayload(BaseModel):
    pixels: List[List[int]]  # 64 x [r,g,b]


def make_app(node: "WebBridgeNode") -> FastAPI:
    app = FastAPI(title="mBuff Web Bridge")

    origins = os.getenv("WEB_ORIGINS", "*")
    origin_list = [o.strip() for o in origins.split(",")] if origins else ["*"]
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origin_list,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @app.get("/api/health")
    def health():
        return {"ok": True, "node": node.get_name()}

    @app.get("/api/status")
    def status():
        return {"ok": True, "connected": True}

    # Movement
    @app.post("/api/move/{direction}")
    def move_direction(direction: str):
        try:
            mv = Move(direction.lower())
        except ValueError:
            raise HTTPException(status_code=400, detail=f"invalid direction: {direction}")
        node._send_move(mv)
        return {"received": True, "action": mv.value}

    @app.post("/api/move/forward")
    def move_forward():
        node._send_move(Move.FORWARD); return {"received": True, "action": Move.FORWARD}

    @app.post("/api/move/backward")
    def move_backward():
        node._send_move(Move.BACKWARD); return {"received": True, "action": Move.BACKWARD}

    @app.post("/api/move/left")
    def move_left():
        node._send_move(Move.LEFT); return {"received": True, "action": Move.LEFT}

    @app.post("/api/move/right")
    def move_right():
        node._send_move(Move.RIGHT); return {"received": True, "action": Move.RIGHT}

    @app.post("/api/move/stop")
    def move_stop():
        node._send_move(Move.STOP); return {"received": True, "action": Move.STOP}

    @app.post("/api/cmd")
    def receive_cmd(cmd: Command):
        node.get_logger().info(f"WEB CMD: action={cmd.action} speed={cmd.speed} note={cmd.note}")
        node._send_move(cmd.action)
        return {"received": True}

    # Sense HAT LEDs
    @app.post("/api/sensehat/leds")
    def set_sensehat_leds(payload: SenseHatPayload):
        if len(payload.pixels) != 64:
            raise HTTPException(status_code=400, detail="pixels must be length 64")
        flat: list[int] = []
        for i, px in enumerate(payload.pixels):
            if not isinstance(px, list) or len(px) != 3:
                raise HTTPException(status_code=400, detail=f"pixel {i} must be [r,g,b]")
            r, g, b = px
            for c in (r, g, b):
                if not isinstance(c, int) or c < 0 or c > 255:
                    raise HTTPException(status_code=400, detail=f"pixel {i} has out-of-range component (0..255)")
            flat.extend([r, g, b])
        node._publish_sensehat(flat)
        return {"ok": True, "count": 64}

    @app.post("/api/sensehat/clear")
    def clear_sensehat():
        node._publish_sensehat([0] * 192)
        return {"ok": True, "cleared": True}

    return app


def start_server_in_thread(app: FastAPI, host: str, port: int):
    config = uvicorn.Config(app, host=host, port=port, log_level="info")
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    return t


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("web_bridge")
        self.move_pub = self.create_publisher(String, "/mbuff/move", 10)
        self.sensehat_pub = self.create_publisher(UInt8MultiArray, "/mbuff/sensehat/leds", 10)

        port = int(os.getenv("WEB_BRIDGE_PORT", "5001"))
        app = make_app(self)
        start_server_in_thread(app, host="0.0.0.0", port=port)
        self.get_logger().info(f"web_bridge up — HTTP listening on :{port}")

        debug = ('-d' in sys.argv) or ('--debug' in sys.argv) or (os.getenv('DEBUG') == '1')
        if debug:
            self._timer = self.create_timer(2.0, self._tick_callback)

    def _tick_callback(self):
        self.get_logger().debug("web_bridge heartbeat…")

    def _send_move(self, action: Move):
        self.get_logger().info(f"WEB MOVE: {action.value}")
        msg = String(); msg.data = action.value
        self.move_pub.publish(msg)

    def _publish_sensehat(self, flat_rgb: list[int]):
        if len(flat_rgb) != 192:
            self.get_logger().warn(f"Sense HAT publish dropped: expected 192 entries, got {len(flat_rgb)}")
            return
        msg = UInt8MultiArray()
        pix = MultiArrayDimension(); pix.label = "pixels"; pix.size = 64; pix.stride = 192
        rgb = MultiArrayDimension(); rgb.label = "rgb";    rgb.size = 3;  rgb.stride = 3
        msg.layout.dim = [pix, rgb]
        msg.data = flat_rgb
        self.sensehat_pub.publish(msg)
        self.get_logger().info("WEB SENSEHAT: published 64x3 RGB frame")


def main():
    rclpy.init()
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
