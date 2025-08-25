import os
import threading
from typing import Literal

import rclpy
from rclpy.node import Node

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn


MoveDir = Literal["forward", "backward", "left", "right", "stop"]

class Command(BaseModel):
    action: MoveDir
    speed: int | None = None
    note: str | None = None


def make_app(node: Node) -> FastAPI:
    app = FastAPI(title="mBuff Web Bridge")

    # --- CORS (adjust origins as needed) ---
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

    # Generic movement route
    @app.post("/api/move/{direction}")
    def move_direction(direction: str):
        d = direction.lower()
        if d not in {"forward", "backward", "left", "right", "stop"}:
            raise HTTPException(status_code=400, detail=f"invalid direction: {direction}")
        node.get_logger().info(f"WEB MOVE: {d}")
        # TODO: publish a ROS2 message here
        return {"received": True, "action": d}

    # Explicit endpoints (if your website calls these exact paths)
    @app.post("/api/move/forward")
    def move_forward():
        node.get_logger().info("WEB MOVE: forward")
        return {"received": True, "action": "forward"}

    @app.post("/api/move/backward")
    def move_backward():
        node.get_logger().info("WEB MOVE: backward")
        return {"received": True, "action": "backward"}

    @app.post("/api/move/left")
    def move_left():
        node.get_logger().info("WEB MOVE: left")
        return {"received": True, "action": "left"}

    @app.post("/api/move/right")
    def move_right():
        node.get_logger().info("WEB MOVE: right")
        return {"received": True, "action": "right"}

    @app.post("/api/move/stop")
    def move_stop():
        node.get_logger().info("WEB MOVE: stop")
        return {"received": True, "action": "stop"}

    # Optional: JSON command body (if you later send speed, etc.)
    @app.post("/api/cmd")
    def receive_cmd(cmd: Command):
        node.get_logger().info(f"WEB CMD: action={cmd.action} speed={cmd.speed} note={cmd.note}")
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
        port_env = os.getenv("WEB_BRIDGE_PORT", "5001")
        try:
            port = int(port_env)
        except ValueError:
            port = 5001

        app = make_app(self)
        start_server_in_thread(app, host="0.0.0.0", port=port)
        self.get_logger().info(f"web_bridge up — HTTP listening on :{port}")

        self._timer = self.create_timer(2.0, self._tick_callback)

    def _tick_callback(self):
        self.get_logger().info("web_bridge heartbeat…")


def main():
    rclpy.init()
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
