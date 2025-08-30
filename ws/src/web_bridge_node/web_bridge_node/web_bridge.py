#!/usr/bin/env python3
import os, sys, threading
from typing import List
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

import rclpy
from rclpy.node import Node

# Local APIs
from .web_bridge_move import MoveAPI
from .web_bridge_sense import SenseAPI
from .web_bridge_camera import CameraAPI


def start_server_in_thread(app: FastAPI, host: str, port: int):
    config = uvicorn.Config(app, host=host, port=port, log_level="info")
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    return t


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__("web_bridge")

        # --- FastAPI app & CORS ---
        app = FastAPI(title="mBuff Web Bridge")
        origins_env = os.getenv("WEB_ORIGINS", "*")
        origins: List[str] = [o.strip() for o in origins_env.split(",")] if origins_env else ["*"]
        app.add_middleware(
            CORSMiddleware,
            allow_origins=origins,
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # --- Subsystems / Routers ---
        self.move_api  = MoveAPI(self, app)     # /api/move/...
        self.sense_api = SenseAPI(self, app)    # /api/sensehat/...
        self.camera_api = CameraAPI(
            self,
            app,
            external_cam=os.getenv("EXTERNAL_CAMERA_URL"),  # e.g. http://<host>:8070/mjpeg
        )

        # Simple health routes here (don’t tie to a sub-API)
        @app.get("/api/health")
        def health():
            return {"ok": True, "node": self.get_name()}

        @app.get("/api/status")
        def status():
            return {
                "ok": True,
                "connected": True,
                "camera_mode": self.camera_api.mode(),
                "camera_upstream": self.camera_api.upstream(),
            }

        port = int(os.getenv("WEB_BRIDGE_PORT", "5001"))
        start_server_in_thread(app, host="0.0.0.0", port=port)
        self.get_logger().info(f"web_bridge up — HTTP listening on :{port}")


def main():
    rclpy.init()
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
