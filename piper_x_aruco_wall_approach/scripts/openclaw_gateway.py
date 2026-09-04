#!/usr/bin/env python3
"""Small OpenClaw gateway that proxies high-level requests to the 8892 API."""

from __future__ import annotations

import argparse
import json
import os
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


ROUTES = {
    "/openclaw/search_marker": "/tools/piper/search-marker",
    "/openclaw/approach_marker": "/tools/piper/approach-marker",
    "/openclaw/touch_marker": "/tools/piper/touch-marker",
    "/openclaw/retract": "/tools/piper/go-nav-pose",
    "/openclaw/stop": "/tools/piper/clear-active-tasks",
}


def json_bytes(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, sort_keys=True).encode("utf-8")


class GatewayHandler(BaseHTTPRequestHandler):
    server_version = "OpenClawGateway/0.1"

    @property
    def api_base(self) -> str:
        return self.server.api_base.rstrip("/")  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.server.verbose:  # type: ignore[attr-defined]
            super().log_message(fmt, *args)

    def send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json_bytes(payload)
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON body: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def proxy(self, method: str, target_path: str, payload: dict[str, Any] | None = None) -> None:
        data = None if payload is None else json_bytes(payload)
        request = urllib.request.Request(
            f"{self.api_base}{target_path}",
            data=data,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=self.server.timeout_s) as response:  # type: ignore[attr-defined]
                body = response.read()
                status = response.status
        except urllib.error.HTTPError as exc:
            body = exc.read()
            status = exc.code
        except urllib.error.URLError as exc:
            self.send_json(
                503,
                {
                    "success": False,
                    "gateway": "openclaw",
                    "api_base": self.api_base,
                    "message": f"lower-level PiPER API unavailable: {exc.reason}",
                },
            )
            return

        try:
            response_payload = json.loads(body.decode("utf-8")) if body else {}
        except json.JSONDecodeError:
            response_payload = {"raw": body.decode("utf-8", errors="replace")}
        if isinstance(response_payload, dict):
            response_payload.setdefault("gateway", "openclaw")
            response_payload.setdefault("proxied_to", target_path)
        self.send_json(status, response_payload if isinstance(response_payload, dict) else {"data": response_payload})

    def do_GET(self) -> None:
        if self.path == "/health":
            self.proxy("GET", "/health")
            return
        if self.path == "/capabilities":
            self.send_json(
                200,
                {
                    "success": True,
                    "gateway": "openclaw",
                    "api_base": self.api_base,
                    "routes": sorted(ROUTES),
                    "rule": "gateway proxies to 8892 and does not own ROS hardware, TF, MoveIt, RTAB-Map, or Nav2",
                },
            )
            return
        self.send_json(404, {"success": False, "message": f"unknown GET route: {self.path}"})

    def do_POST(self) -> None:
        target_path = ROUTES.get(self.path)
        if target_path is None:
            self.send_json(404, {"success": False, "message": f"unknown POST route: {self.path}"})
            return
        try:
            payload = self.read_json_body()
        except ValueError as exc:
            self.send_json(400, {"success": False, "message": str(exc)})
            return
        if self.path == "/openclaw/stop":
            payload.setdefault("clear_command_lock", True)
            payload.setdefault("clear_ros_tasks", False)
        self.proxy("POST", target_path, payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("OPENCLAW_GATEWAY_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("OPENCLAW_GATEWAY_PORT", "8893")))
    parser.add_argument("--api-base", default=os.environ.get("PIPER_TOUCH_API_URL", "http://127.0.0.1:8892"))
    parser.add_argument("--timeout-s", type=float, default=float(os.environ.get("OPENCLAW_GATEWAY_TIMEOUT_S", "10.0")))
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), GatewayHandler)
    server.api_base = args.api_base
    server.timeout_s = args.timeout_s
    server.verbose = args.verbose
    print(f"OpenClaw gateway listening on http://{args.host}:{args.port}")
    print(f"Forwarding lower-level PiPER calls to {args.api_base}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
