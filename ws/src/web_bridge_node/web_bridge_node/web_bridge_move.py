from enum import Enum
from fastapi import FastAPI, HTTPException
from std_msgs.msg import String

class Move(str, Enum):
    FORWARD  = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT     = "LEFT"
    RIGHT    = "RIGHT"
    STOP     = "STOP"
    SPEEDUP  = "SPEEDUP"
    SPEEDDOWN= "SPEEDDOWN"
    SPIN_LEFT       = "SPIN_LEFT"
    SPIN_RIGHT      = "SPIN_RIGHT"
    SPIN_BACK_LEFT  = "SPIN_BACK_LEFT"
    SPIN_BACK_RIGHT = "SPIN_BACK_RIGHT"

class MoveAPI:
    def __init__(self, node, app: FastAPI):
        self.node = node
        self.pub = node.create_publisher(String, "/mbuff/move", 10)

        # Optional simple status for your UI getStatus()
        @app.get("/api/status")
        def status():
            return {"ok": True}

        # Generic move (accepts lowercase paths; converts to Enum)
        @app.post("/api/move/{direction}")
        def move_direction(direction: str):
            try:
                mv = Move(direction.upper())
            except ValueError:
                raise HTTPException(status_code=400, detail=f"invalid direction: {direction}")
            self._send_move(mv)
            return {"received": True, "action": mv.value}

        # Basic moves
        @app.post("/api/move/forward")
        def move_forward():   self._send_move(Move.FORWARD);   return {"received": True, "action": Move.FORWARD}
        @app.post("/api/move/backward")
        def move_backward():  self._send_move(Move.BACKWARD);  return {"received": True, "action": Move.BACKWARD}
        @app.post("/api/move/left")
        def move_left():      self._send_move(Move.LEFT);      return {"received": True, "action": Move.LEFT}
        @app.post("/api/move/right")
        def move_right():     self._send_move(Move.RIGHT);     return {"received": True, "action": Move.RIGHT}
        @app.post("/api/move/stop")
        def move_stop():      self._send_move(Move.STOP);      return {"received": True, "action": Move.STOP}

        # Speed
        @app.post("/api/move/speedup")
        def move_speedup():   self._send_move(Move.SPEEDUP);   return {"received": True, "action": Move.SPEEDUP}
        @app.post("/api/move/speeddown")
        def move_speeddown(): self._send_move(Move.SPEEDDOWN); return {"received": True, "action": Move.SPEEDDOWN}

        # Rotate / spin
        @app.post("/api/rotate/topleft")
        def rotate_topleft():      self._send_move(Move.SPIN_LEFT);        return {"received": True, "action": Move.SPIN_LEFT}
        @app.post("/api/rotate/topright")
        def rotate_topright():     self._send_move(Move.SPIN_RIGHT);       return {"received": True, "action": Move.SPIN_RIGHT}
        @app.post("/api/rotate/bottomleft")
        def rotate_bottomleft():   self._send_move(Move.SPIN_BACK_LEFT);   return {"received": True, "action": Move.SPIN_BACK_LEFT}
        @app.post("/api/rotate/bottomright")
        def rotate_bottomright():  self._send_move(Move.SPIN_BACK_RIGHT);  return {"received": True, "action": Move.SPIN_BACK_RIGHT}

        # Dance (matches website: /api/move/dance/{id})
        @app.post("/api/dance/{idx}")
        def dance(idx: int):
            if idx not in (1, 2, 3, 4):
                raise HTTPException(status_code=400, detail="dance id must be 1..4")
            self._publish(f"DANCE{idx}")
            return {"received": True, "action": f"DANCE{idx}"}

        # Song (/api/song/{id})
        @app.post("/api/song/{idx}")
        def song(idx: int):
            if idx not in (1, 2, 3, 4):
                raise HTTPException(status_code=400, detail="song id must be 1..4")
            self._publish(f"SONG{idx}")
            return {"received": True, "action": f"SONG{idx}"}

        # Say + Speak (both call /api/say/{id} per your website)
        @app.post("/api/say/{idx}")
        def say(idx: int):
            if idx not in (1, 2, 3, 4, 5, 6):
                raise HTTPException(status_code=400, detail="say id must be 1..6")
            self._publish(f"SAYING{idx}")
            return {"received": True, "action": f"SAYING{idx}"}

    # ---- internals ----
    def _publish(self, text: str) -> None:
        self.node.get_logger().info(f"WEB TX: {text}")
        msg = String(); msg.data = text
        self.pub.publish(msg)

    def _send_move(self, mv: Move) -> None:
        self._publish(mv.value)
