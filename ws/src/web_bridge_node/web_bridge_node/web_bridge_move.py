from enum import Enum
from fastapi import FastAPI, HTTPException
from std_msgs.msg import String

class Move(str, Enum):
    FORWARD  = "forward"
    BACKWARD = "backward"
    LEFT     = "left"
    RIGHT    = "right"
    STOP     = "stop"

class MoveAPI:
    def __init__(self, node, app: FastAPI):
        self.node = node
        # Publish directly to /mbuff/move (simple; you can remap if needed)
        self.pub = node.create_publisher(String, "/mbuff/move", 10)

        @app.post("/api/move/{direction}")
        def move_direction(direction: str):
            try:
                mv = Move(direction.lower())
            except ValueError:
                raise HTTPException(status_code=400, detail=f"invalid direction: {direction}")
            self._send(mv)
            return {"received": True, "action": mv.value}

        @app.post("/api/move/forward")
        def move_forward():  self._send(Move.FORWARD);  return {"received": True, "action": Move.FORWARD}
        @app.post("/api/move/backward")
        def move_backward(): self._send(Move.BACKWARD); return {"received": True, "action": Move.BACKWARD}
        @app.post("/api/move/left")
        def move_left():     self._send(Move.LEFT);     return {"received": True, "action": Move.LEFT}
        @app.post("/api/move/right")
        def move_right():    self._send(Move.RIGHT);    return {"received": True, "action": Move.RIGHT}
        @app.post("/api/move/stop")
        def move_stop():     self._send(Move.STOP);     return {"received": True, "action": Move.STOP}

    def _send(self, mv: Move):
        self.node.get_logger().info(f"WEB MOVE: {mv.value}")
        msg = String(); msg.data = mv.value
        self.pub.publish(msg)
