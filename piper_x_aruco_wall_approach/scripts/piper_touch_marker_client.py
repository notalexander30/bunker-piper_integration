#!/usr/bin/env python3

import argparse
import json
import os
import sys
import urllib.error
import urllib.request


def request_json(method, url, payload=None, token=None, timeout_s=120.0):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if payload is not None:
        headers["Content-Type"] = "application/json"
    if token:
        headers["Authorization"] = f"Bearer {token}"
    request = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        response_context = (
            urllib.request.urlopen(request)
            if timeout_s is None
            else urllib.request.urlopen(request, timeout=timeout_s)
        )
        with response_context as response:
            body = response.read().decode("utf-8")
            return response.status, json.loads(body)
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8")
        try:
            parsed = json.loads(body)
        except json.JSONDecodeError:
            parsed = {"detail": body}
        return exc.code, parsed


def print_result(status_code, body):
    print(json.dumps(body, indent=2, sort_keys=True))
    if status_code >= 400 or body.get("success") is False:
        return 1
    return 0


def task_payload(args):
    return {
        "execute": args.execute,
        "arm": args.arm,
        "pre_clearance_m": args.pre_clearance,
        "final_clearance_m": args.final_clearance,
        "retract_after": args.retract,
        "retract_distance_m": args.retract_distance,
        "final_velocity_scaling": args.final_velocity_scaling,
        "return_home_after": args.return_home_after,
        "home_duration_s": args.home_duration,
    }


def home_payload(args):
    return {
        "execute": args.execute,
        "arm": args.arm,
        "duration_s": args.duration,
    }


def save_home_payload(args):
    return {
        "pose_name": args.pose_name,
    }


def main():
    parser = argparse.ArgumentParser(description="Operator client for the PiPER-X marker API.")
    parser.add_argument("--base-url", default=os.environ.get("PIPER_TOUCH_API_URL", "http://127.0.0.1:8892"))
    parser.add_argument("--token", default=os.environ.get("PIPER_TOUCH_API_TOKEN"))
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("health")

    for name in ("approach", "touch"):
        task = subparsers.add_parser(name)
        task.add_argument("--execute", action="store_true")
        task.add_argument("--arm", choices=["front", "rear"], default="front")
        task.add_argument("--pre-clearance", type=float, default=0.05)
        task.add_argument("--final-clearance", type=float, default=0.005)
        task.add_argument("--retract", action="store_true")
        task.add_argument("--retract-distance", type=float, default=0.05)
        task.add_argument("--final-velocity-scaling", type=float, default=0.16)
        task.add_argument("--return-home-after", action="store_true")
        task.add_argument("--home-duration", type=float, default=6.0)

    search = subparsers.add_parser("search")
    search.add_argument("--execute", action="store_true")
    search.add_argument("--arm", choices=["front", "rear"], default="front")

    home = subparsers.add_parser("home")
    home.add_argument("--execute", action="store_true")
    home.add_argument("--arm", choices=["front", "rear"], default="front")
    home.add_argument("--duration", type=float, default=6.0)

    previous = subparsers.add_parser("previous")
    previous.add_argument("--execute", action="store_true")
    previous.add_argument("--arm", choices=["front", "rear"], default="front")
    previous.add_argument("--duration", type=float, default=6.0)

    nav_pose = subparsers.add_parser("nav-pose")
    nav_pose.add_argument("--execute", action="store_true")
    nav_pose.add_argument("--arm", choices=["front", "rear"], default="front")
    nav_pose.add_argument("--duration", type=float, default=6.0)

    save_home = subparsers.add_parser("save-home")
    save_home.add_argument("--pose-name", default="home")
    subparsers.add_parser("save-previous")

    args = parser.parse_args()
    base_url = args.base_url.rstrip("/")
    if args.command == "health":
        return print_result(*request_json("GET", f"{base_url}/health", token=args.token))
    if args.command == "home":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/go-home",
                payload=home_payload(args),
                token=args.token,
            )
        )
    if args.command == "previous":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/go-previous",
                payload=home_payload(args),
                token=args.token,
            )
        )
    if args.command == "nav-pose":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/go-nav-pose",
                payload=home_payload(args),
                token=args.token,
            )
        )
    if args.command == "save-home":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/save-home",
                payload=save_home_payload(args),
                token=args.token,
            )
        )
    if args.command == "save-previous":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/save-previous",
                payload={},
                token=args.token,
            )
        )
    if args.command == "search":
        return print_result(
            *request_json(
                "POST",
                f"{base_url}/tools/piper/search-marker",
                payload={"execute": args.execute, "arm": args.arm},
                token=args.token,
                timeout_s=None,
            )
        )
    endpoint = "approach-marker" if args.command == "approach" else "touch-marker"
    return print_result(
        *request_json(
            "POST",
            f"{base_url}/tools/piper/{endpoint}",
            payload=task_payload(args),
            token=args.token,
            timeout_s=None,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
