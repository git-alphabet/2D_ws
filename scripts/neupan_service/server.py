#!/usr/bin/env python3
"""NeuPAN runtime service for dual-container deployment.

Runs in a dedicated Python 3.12 container and exposes a small HTTP API:
- GET  /health
- POST /init
- POST /reset
- POST /set_initial_path
- POST /update_initial_path_from_goal
- POST /forward
"""

from __future__ import annotations

import argparse
import json
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

try:
    from neupan.neupan import neupan as NeupanClass
except Exception as exc:  # pragma: no cover
    raise RuntimeError(f"failed to import neupan: {exc}") from exc


class ServiceState:
    def __init__(self) -> None:
        self.instance: Any | None = None
        self.config_path: str | None = None

    @property
    def initialized(self) -> bool:
        return self.instance is not None


STATE = ServiceState()


def _json_response(handler: BaseHTTPRequestHandler, code: int, payload: dict[str, Any]) -> None:
    body = json.dumps(payload, ensure_ascii=True).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _to_state_array(xyt: list[float]) -> np.ndarray:
    arr = np.asarray(xyt, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"expected state shape (3,), got {arr.shape}")
    return arr.reshape(3, 1)


def _to_obstacles(points: list[list[float]] | None) -> np.ndarray | None:
    if not points:
        return None
    arr = np.asarray(points, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError(f"expected obstacles Nx2, got {arr.shape}")
    return arr.T


def _as_list_or_none(value: Any) -> Any:
    if value is None:
        return None
    if hasattr(value, "detach") and callable(getattr(value, "detach")):
        value = value.detach()
    if hasattr(value, "cpu") and callable(getattr(value, "cpu")):
        value = value.cpu()
    if hasattr(value, "tolist") and callable(getattr(value, "tolist")):
        try:
            return value.tolist()
        except Exception:
            pass
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        out = []
        for item in value:
            out.append(_as_list_or_none(item))
        return out
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def _safe_float(value: Any, default: float) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except Exception:
        return default


class NeupanHandler(BaseHTTPRequestHandler):
    server_version = "NeuPANService/1.0"

    def log_message(self, format: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            _json_response(
                self,
                HTTPStatus.OK,
                {
                    "ok": True,
                    "initialized": STATE.initialized,
                    "config_path": STATE.config_path,
                },
            )
            return
        _json_response(self, HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        try:
            raw = self.rfile.read(int(self.headers.get("Content-Length", "0")))
            data = json.loads(raw.decode("utf-8") or "{}")
        except Exception as exc:
            _json_response(self, HTTPStatus.BAD_REQUEST, {"ok": False, "error": f"invalid json: {exc}"})
            return

        try:
            if self.path == "/init":
                self._handle_init(data)
            elif self.path == "/reset":
                self._handle_reset()
            elif self.path == "/set_initial_path":
                self._handle_set_initial_path(data)
            elif self.path == "/update_initial_path_from_goal":
                self._handle_update_initial_path_from_goal(data)
            elif self.path == "/forward":
                self._handle_forward(data)
            else:
                _json_response(self, HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
        except Exception as exc:
            _json_response(
                self,
                HTTPStatus.INTERNAL_SERVER_ERROR,
                {
                    "ok": False,
                    "error": str(exc),
                    "traceback": traceback.format_exc(limit=4),
                },
            )

    def _require_instance(self) -> Any:
        if STATE.instance is None:
            raise RuntimeError("neupan not initialized, call /init first")
        return STATE.instance

    def _handle_init(self, data: dict[str, Any]) -> None:
        config_path = str(data.get("config_path", "")).strip()
        if not config_path:
            raise ValueError("config_path is required")

        cfg = Path(config_path)
        if not cfg.exists():
            raise FileNotFoundError(f"config_path not found: {config_path}")

        dune_model_path = str(data.get("dune_model_path", "")).strip()
        kwargs: dict[str, Any] = {}
        if dune_model_path:
            kwargs["pan"] = {"dune_checkpoint": dune_model_path}

        STATE.instance = NeupanClass.init_from_yaml(config_path, **kwargs)
        STATE.config_path = config_path
        _json_response(self, HTTPStatus.OK, {"ok": True, "initialized": True})

    def _handle_reset(self) -> None:
        instance = self._require_instance()
        instance.reset()
        _json_response(self, HTTPStatus.OK, {"ok": True})

    def _handle_set_initial_path(self, data: dict[str, Any]) -> None:
        instance = self._require_instance()
        raw_path = data.get("path", [])
        if not isinstance(raw_path, list):
            raise ValueError("path must be a list")

        converted = []
        for idx, wp in enumerate(raw_path):
            arr = np.asarray(wp, dtype=float)
            if arr.shape != (4,):
                raise ValueError(f"path[{idx}] must be [x,y,theta,gear], got {arr.shape}")
            converted.append(arr.reshape(4, 1))

        instance.set_initial_path(converted)
        _json_response(self, HTTPStatus.OK, {"ok": True, "count": len(converted)})

    def _handle_update_initial_path_from_goal(self, data: dict[str, Any]) -> None:
        instance = self._require_instance()
        start = _to_state_array(data.get("start", []))
        goal = _to_state_array(data.get("goal", []))
        instance.update_initial_path_from_goal(start, goal)
        _json_response(self, HTTPStatus.OK, {"ok": True})

    def _handle_forward(self, data: dict[str, Any]) -> None:
        instance = self._require_instance()
        robot_state = _to_state_array(data.get("robot_state", []))
        obstacles = _to_obstacles(data.get("obstacles", []))

        action, info = instance.forward(robot_state, obstacles, None)

        payload = {
            "ok": True,
            "action": _as_list_or_none(action),
            "stop": bool(info.get("stop", False)),
            "arrive": bool(info.get("arrive", False)),
            "opt_state_list": _as_list_or_none(info.get("opt_state_list")),
            "ref_state_list": _as_list_or_none(info.get("ref_state_list")),
            "initial_path": _as_list_or_none(getattr(instance, "initial_path", None)),
            "dune_points": _as_list_or_none(getattr(instance, "dune_points", None)),
            "nrmp_points": _as_list_or_none(getattr(instance, "nrmp_points", None)),
            "robot": {
                "shape": str(getattr(instance.robot, "shape", "rectangle")),
                "kinematics": str(getattr(instance.robot, "kinematics", "diff")),
                "length": _safe_float(getattr(instance.robot, "length", 0.5), 0.5),
                "width": _safe_float(getattr(instance.robot, "width", 0.3), 0.3),
                "wheelbase": _safe_float(getattr(instance.robot, "wheelbase", 0.3), 0.3),
            },
        }
        _json_response(self, HTTPStatus.OK, payload)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NeuPAN runtime HTTP service")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=18080)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), NeupanHandler)
    print(f"[neupan-service] listening on {args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
