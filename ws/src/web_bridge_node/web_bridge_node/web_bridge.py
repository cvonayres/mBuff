import os, sys, threading
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        # TODO: expand with real telemetry later
        return {"ok": True, "connected": True}

    # Generic route matches the website's pattern too
    @app.post("/api/move/{direction}")
    def move_direction(direction: str):
        try:
            mv = Move(direction.lower())
        except ValueError:
            raise HTTPException(status_code=400, detail=f"invalid direction: {direction}")
        node._send_move(mv)
        return {"received": True, "action": mv.value}

    # Explicit routes
    @app.post("/api/move/forward")
    def move_forward():
        node._send_move(Move.FORWARD)
        return {"received": True, "action": Move.FORWARD}

    @app.post("/api/move/backward")
    def move_backward():
        node._send_move(Move.BACKWARD)
        return {"received": True, "action": Move.BACKWARD}

    @app.post("/api/move/left")
    def move_left():
        node._send_move(Move.LEFT)
        return {"received": True, "action": Move.LEFT}

    @app.post("/api/move/right")
    def move_right():
        node._send_move(Move.RIGHT)
        return {"received": True, "action": Move.RIGHT}

    @app.post("/api/move/stop")
    def move_stop():
        node._send_move(Move.STOP)
        return {"received": True, "action": Move.STOP}

    @app.post("/api/cmd")
    def receive_cmd(cmd: Command):
        node.get_logger().info(f"WEB CMD: action={cmd.action} speed={cmd.speed} note={cmd.note}")
        node._send_move(cmd.action)  # optional: still publish
        return {"received": True}

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

        # Publisher (so endpoints can use it)
        self.move_pub = self.create_publisher(String, "/mbuff/move", 10)

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
        msg = String()
        msg.data = action.value
        self.move_pub.publish(msg)


def main():
    rclpy.init()
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
