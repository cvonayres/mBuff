#!/usr/bin/env python3
# web_bridge_node/web_bridge_camera.py

from __future__ import annotations
import asyncio
import time
import os
from typing import Optional, AsyncGenerator, List
import urllib.parse as urlparse

import httpx
from fastapi import FastAPI, HTTPException, Response
from fastapi.responses import StreamingResponse
from sensor_msgs.msg import CompressedImage
import threading


def _replace_path(url: str, new_path: str) -> str:
    """Replace the path of a URL with new_path."""
    try:
        u = urlparse.urlsplit(url)
        return urlparse.urlunsplit((u.scheme, u.netloc, new_path, u.query, u.fragment))
    except Exception:
        return url


class CameraAPI:
    """
    Handles camera HTTP endpoints for the web bridge.
    - If EXTERNAL camera URL is set (e.g. http://host.docker.internal:8088/mjpeg),
      proxy/rechunk that stream and provide a snapshot endpoint.
    - Otherwise, serve frames from a ROS CompressedImage subscription.
    """

    def __init__(self, node, app: FastAPI, external_cam: Optional[str]):
        self.node = node
        self.external_cam = external_cam
        self._latest_frame: Optional[bytes] = None
        self._lock = threading.Lock()

        if self.external_cam:
            self.node.get_logger().info(f"Camera mode: PROXY -> {self.external_cam}")
        else:
            self.node.get_logger().info("Camera mode: ROS topic '/mbuff/camera/image_raw'")
            node.create_subscription(CompressedImage, "/mbuff/camera/image_raw", self._on_cam, 10)

        # --- Simple probes (never hang) ---
        @app.head("/api/camera/feed")
        def _head_feed():        return Response(status_code=200)

        @app.head("/api/camera/feed/rechunk")
        def _head_rechunk():     return Response(status_code=200)

        @app.get("/api/camera/proxy")
        async def proxy_check():
            if not self.external_cam:
                raise HTTPException(status_code=404, detail="proxy disabled (ROS camera mode)")
            # Quick HEAD-ish check using a very short timeout
            try:
                async with httpx.AsyncClient(timeout=httpx.Timeout(connect=2.0, read=2.0, write=2.0, pool=2.0)) as c:
                    r = await c.get(self.external_cam, headers={"Accept": "multipart/x-mixed-replace"}, follow_redirects=True)
                    ok = (r.status_code == 200)
                    ct = r.headers.get("content-type", "")
                    await r.aclose()
                if not ok:
                    raise HTTPException(status_code=502, detail=f"upstream status {r.status_code}")
                return {"ok": True, "status": 200, "content_type": ct, "upstream": self.external_cam}
            except httpx.HTTPError as e:
                raise HTTPException(status_code=502, detail=f"proxy error: {e!s}")

        # --- Debug helpers (bounded, non-hanging) ---
        @app.get("/api/camera/debug/targets")
        def debug_targets():
            cands = self._candidates()
            return {"candidates": cands, "configured": self.external_cam}

        @app.get("/api/camera/debug/tcp")
        async def debug_tcp():
            results = []
            for u in self._candidates():
                try:
                    parsed = urlparse.urlsplit(u)
                    host = parsed.hostname
                    port = parsed.port or 80
                    self.node.get_logger().info(f"[camera] tcp connect {host}:{port}")
                    start = time.time()
                    reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), timeout=2.0)
                    writer.close()
                    with contextlib.suppress(Exception):
                        await writer.wait_closed()
                    results.append({"host": host, "port": port, "url": u, "ok": True, "ms": int((time.time()-start)*1000)})
                except Exception as e:
                    results.append({"host": host, "port": port, "url": u, "ok": False, "error": repr(e)})
            return {"tcp": results}

        @app.get("/api/camera/debug/httpx")
        async def debug_httpx():
            """Try to pull a small chunk quickly from upstream (never hang)."""
            if not self.external_cam:
                return {"httpx": []}
            results = []
            for u in self._candidates():
                try:
                    t0 = time.time()
                    async with httpx.AsyncClient(timeout=httpx.Timeout(connect=2.0, read=3.0, write=2.0, pool=2.0)) as c:
                        async with c.stream("GET", u, headers={"Accept": "multipart/x-mixed-replace"}, follow_redirects=True) as r:
                            ct = r.headers.get("content-type", "")
                            status = r.status_code
                            # read ~16KB at most
                            got = 0
                            async for chunk in r.aiter_bytes():
                                got += len(chunk)
                                if got >= 16384:
                                    break
                    results.append({"url": u, "ok": True, "status": status, "content_type": ct,
                                    "bytes_sampled": got, "ms": int((time.time()-t0)*1000)})
                except httpx.ReadTimeout:
                    results.append({"url": u, "ok": False, "error": "ReadTimeout"})
                except httpx.HTTPError as e:
                    results.append({"url": u, "ok": False, "error": repr(e)})
            return {"httpx": results}

        @app.get("/api/camera/debug/snap")
        async def debug_snap():
            """Try upstream /snapshot.jpg quickly (preferred fast path)."""
            if not self.external_cam:
                return {"snap": []}
            results = []
            for u in self._candidates():
                snap = self._derive_snapshot_url(u)
                try:
                    t0 = time.time()
                    async with httpx.AsyncClient(timeout=httpx.Timeout(connect=2.0, read=3.0, write=2.0, pool=2.0)) as c:
                        r = await c.get(snap, follow_redirects=True)
                        ok = (r.status_code == 200 and r.headers.get("content-type", "").startswith("image/"))
                        size = len(r.content) if ok else 0
                    results.append({"url": snap, "ok": ok, "status": r.status_code, "bytes": size,
                                    "content_type": r.headers.get("content-type", ""), "ms": int((time.time()-t0)*1000)})
                except httpx.HTTPError as e:
                    results.append({"url": snap, "ok": False, "error": repr(e)})
            return {"snap": results}

        # --- Main endpoints ---
        @app.get("/api/camera/snapshot.jpg")
        async def snapshot():
            """
            First try upstream /snapshot.jpg with a short timeout (fast, single image).
            If that fails, fall back to sniffing the MJPEG for the first JPEG with a bounded wait.
            """
            # Proxy mode
            if self.external_cam:
                # 1) Preferred: direct snapshot
                snap = self._derive_snapshot_url(self.external_cam)
                try:
                    async with httpx.AsyncClient(timeout=httpx.Timeout(connect=2.0, read=3.0, write=2.0, pool=2.0)) as c:
                        r = await c.get(snap, follow_redirects=True)
                        if r.status_code == 200 and r.headers.get("content-type", "").startswith("image/"):
                            return Response(content=r.content, media_type=r.headers.get("content-type", "image/jpeg"))
                except httpx.HTTPError as e:
                    self.node.get_logger().warning(f"[camera] direct snapshot failed: {e!s}")

                # 2) Fallback: read first frame from stream with a firm cap (~3s)
                try:
                    frame = await asyncio.wait_for(self._first_frame_from_stream(), timeout=3.0)
                    return Response(content=frame, media_type="image/jpeg")
                except asyncio.TimeoutError:
                    raise HTTPException(status_code=504, detail="snapshot timeout (no frame in 3s)")
                except HTTPException:
                    raise
                except Exception as e:
                    raise HTTPException(status_code=502, detail=f"snapshot error: {e!s}")

            # ROS mode
            with self._lock:
                data = self._latest_frame
            if not data:
                raise HTTPException(status_code=404, detail="no frame")
            return Response(content=data, media_type="image/jpeg")

        @app.get("/api/camera/feed")
        async def camera_feed():
            # Raw passthrough of upstream stream (works with curl; some browsers may stall)
            if self.external_cam:
                return StreamingResponse(
                    self._proxy_passthru(),
                    media_type="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-cache", "Pragma": "no-cache", "X-Accel-Buffering": "no"},
                )
            # ROS fallback
            return StreamingResponse(self._ros_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

        @app.get("/api/camera/feed/rechunk")
        async def camera_feed_rechunk():
            # Emit our own --frame parts (browser-friendly)
            if self.external_cam:
                return StreamingResponse(
                    self._proxy_rechunk(),
                    media_type="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-cache", "Pragma": "no-cache", "X-Accel-Buffering": "no"},
                )
            # ROS fallback
            return StreamingResponse(self._ros_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ---------- helpers ----------
    def mode(self) -> str:
        return "proxy" if self.external_cam else "ros"

    def upstream(self) -> Optional[str]:
        return self.external_cam

    def _candidates(self) -> List[str]:
        if not self.external_cam:
            return []
        # Try configured URL only (it’s known-good in your setup).
        return [self.external_cam]

    def _derive_snapshot_url(self, upstream_mjpeg: str) -> str:
        # If the relay follows our example, /snapshot.jpg exists alongside /mjpeg
        return _replace_path(upstream_mjpeg, "/snapshot.jpg")

    def _on_cam(self, msg: CompressedImage):
        with self._lock:
            self._latest_frame = bytes(msg.data)

    def _stream_timeout(self) -> httpx.Timeout:
        # Long read for streaming; short connect/write
        return httpx.Timeout(connect=3.0, read=300.0, write=3.0, pool=5.0)

    async def _first_frame_from_stream(self) -> bytes:
        async for jpg in self._proxy_frames():
            return jpg
        raise HTTPException(status_code=502, detail="no frames yielded")

    # ---- proxy implementations ----
    async def _proxy_passthru(self) -> AsyncGenerator[bytes, None]:
        """Pass upstream bytes as-is."""
        try:
            async with httpx.AsyncClient(timeout=self._stream_timeout()) as c:
                async with c.stream("GET", self.external_cam, headers={"Accept": "multipart/x-mixed-replace"}, follow_redirects=True) as r:
                    if r.status_code != 200:
                        raise HTTPException(status_code=r.status_code, detail="upstream camera error")
                    async for chunk in r.aiter_bytes():
                        if chunk:
                            yield chunk
        except httpx.HTTPError as e:
            self.node.get_logger().error(f"[camera] passthru error: {e!s}")

    async def _proxy_rechunk(self) -> AsyncGenerator[bytes, None]:
        """Parse upstream bytes and emit our own --frame parts."""
        boundary = b"--frame"
        async for jpg in self._proxy_frames():
            yield (
                boundary + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpg)).encode("ascii") + b"\r\n\r\n"
                + jpg + b"\r\n"
            )

    async def _proxy_frames(self) -> AsyncGenerator[bytes, None]:
        """
        JPEG sniffer: find SOI/EOI in the upstream stream and yield complete JPEGs.
        Works even if upstream boundary is unexpected.
        """
        import contextlib
        buf = bytearray()
        in_jpeg = False
        u = self.external_cam
        if not u:
            return
        try:
            async with httpx.AsyncClient(timeout=self._stream_timeout()) as c:
                async with c.stream("GET", u, headers={"Accept": "multipart/x-mixed-replace"}, follow_redirects=True) as r:
                    if r.status_code != 200:
                        raise HTTPException(status_code=r.status_code, detail="upstream camera error")
                    async for chunk in r.aiter_bytes():
                        if not chunk:
                            continue
                        buf.extend(chunk)
                        # find JPEGs (SOI=FFD8 ... EOI=FFD9)
                        while True:
                            if not in_jpeg:
                                i = buf.find(b"\xff\xd8")
                                if i < 0:
                                    break
                                del buf[:i]
                                in_jpeg = True
                            j = buf.find(b"\xff\xd9")
                            if j < 0:
                                break
                            j += 2
                            jpg = bytes(buf[:j])
                            del buf[:j]
                            in_jpeg = False
                            yield jpg
        except httpx.HTTPError as e:
            self.node.get_logger().error(f"[camera] frames error {u}: {e!s}")

    async def _ros_frames(self) -> AsyncGenerator[bytes, None]:
        boundary = b"--frame"
        while True:
            data = None
            with self._lock:
                if self._latest_frame:
                    data = self._latest_frame
            if data:
                yield (
                    boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(data)).encode("ascii") + b"\r\n\r\n"
                    + data + b"\r\n"
                )
            await asyncio.sleep(0.1)  # ~10 FPS
