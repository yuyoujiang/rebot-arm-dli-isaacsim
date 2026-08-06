#!/usr/bin/env python3
"""Read the reBot Arm 102 leader and publish Seeed-compatible Isaac UDP."""

from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import socket
import sys
import time
from pathlib import Path

from teleop_protocol import encode_payload, leader_action_to_payload


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "teleop_config.json"
_running = True


def _stop(_signum, _frame) -> None:
    global _running
    _running = False


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--port", help="Leader serial port, for example /dev/ttyUSB0")
    parser.add_argument("--host")
    parser.add_argument("--udp-port", type=int)
    parser.add_argument("--rate", type=float)
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Explicit test mode: send a fixed safe pose without connecting hardware",
    )
    parser.add_argument(
        "--mock-profile",
        choices=("hold", "grasp"),
        default="hold",
        help="Mock test data; only used with --mock",
    )
    return parser.parse_args()


def _resolve_serial_port(requested: str | None, default_port: str) -> str:
    if requested:
        return requested
    devices = sorted(glob.glob("/dev/ttyUSB*"))
    if default_port in devices:
        return default_port
    if len(devices) == 1:
        return devices[0]
    if not devices:
        raise RuntimeError(
            "No /dev/ttyUSB* device was detected for the reBot 102 leader. "
            "Connect the USB-UART adapter, then run `ls /dev/ttyUSB*`."
        )
    raise RuntimeError(
        f"Multiple leader serial ports were detected: {devices}. "
        "Select one explicitly with --leader-port/--port."
    )


def _lerp(start: float, end: float, alpha: float) -> float:
    alpha = min(1.0, max(0.0, alpha))
    alpha = alpha * alpha * (3.0 - 2.0 * alpha)
    return start + alpha * (end - start)


def _mock_action(elapsed: float, profile: str) -> dict[str, float]:
    # Leader-space values: lift=+X maps to sim joint2=-X; elbow is 1:1.
    shoulder_lift = 45.0
    elbow_flex = -45.0
    gripper = 45.0
    if profile == "grasp":
        if elapsed < 0.5:
            pass
        elif elapsed < 1.5:
            shoulder_lift = _lerp(45.0, 80.0, (elapsed - 0.5) / 1.0)
        elif elapsed < 2.3:
            shoulder_lift = _lerp(80.0, 90.0, (elapsed - 1.5) / 0.8)
        elif elapsed < 3.0:
            shoulder_lift = 90.0
            gripper = _lerp(45.0, 0.0, (elapsed - 2.3) / 0.7)
        elif elapsed < 4.3:
            shoulder_lift = _lerp(90.0, 70.0, (elapsed - 3.0) / 1.3)
            gripper = 0.0
        else:
            shoulder_lift = 70.0
            gripper = 0.0

    return {
        "shoulder_pan.pos": 0.0,
        "shoulder_lift.pos": shoulder_lift,
        "elbow_flex.pos": elbow_flex,
        "wrist_flex.pos": 0.0,
        "wrist_yaw.pos": 0.0,
        "wrist_roll.pos": 0.0,
        "gripper.pos": gripper,
    }


def main() -> int:
    args = _parse_args()
    config = json.loads(args.config.resolve().read_text(encoding="utf-8"))
    udp = config["udp"]
    host = args.host or udp["host"]
    udp_port = args.udp_port or int(udp["port"])
    rate = args.rate or float(udp["send_hz"])
    if rate <= 0:
        raise ValueError("The send rate must be greater than zero")

    leader = None
    if args.mock:
        print("[leader] MOCK sender enabled; real hardware is not being accessed")
    else:
        serial_port = _resolve_serial_port(
            args.port, str(config["leader"]["default_port"])
        )
        if not os.access(serial_port, os.R_OK | os.W_OK):
            raise PermissionError(
                f"The current user cannot read or write the serial port: {serial_port}\n"
                f"Run this command first: sudo chmod 666 {serial_port}"
            )
        try:
            from lerobot_teleoperator_rebot_arm_102 import (
                RebotArm102Leader,
                RebotArm102LeaderConfig,
            )
        except ImportError as exc:
            raise RuntimeError(
                "The current Python environment does not provide "
                "lerobot-teleoperator-rebot-arm-102. Use "
                "/home/seeed/rebot_lerobot/.venv/bin/python"
            ) from exc

        leader_config = RebotArm102LeaderConfig(
            port=serial_port,
            id=str(config["leader"]["id"]),
        )
        leader = RebotArm102Leader(leader_config)
        if not leader.is_calibrated:
            raise RuntimeError(
                "No calibration file was found for this leader. Follow the "
                "Seeed guide and run "
                "`lerobot-calibrate --teleop.type=rebot_arm_102_leader "
                f"--teleop.port={serial_port} "
                f"--teleop.id={config['leader']['id']}`"
            )
        leader.connect(calibrate=False)
        print(f"[leader] Connected to {serial_port}; calibration ID={config['leader']['id']}")

    output = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / rate
    sequence = 0
    started_at = time.perf_counter()
    next_tick = time.perf_counter()
    print(f"[leader] Sending to udp://{host}:{udp_port} at {rate:.1f} Hz")
    try:
        while _running:
            action = (
                _mock_action(time.perf_counter() - started_at, args.mock_profile)
                if args.mock
                else dict(leader.get_action())
            )
            payload = leader_action_to_payload(
                action,
                config,
                sequence=sequence,
                timestamp=time.time(),
            )
            output.sendto(encode_payload(payload), (host, udp_port))
            if sequence % max(int(rate), 1) == 0:
                q_deg = [
                    round(value * 180.0 / 3.141592653589793, 2)
                    for value in payload["joint_positions"]
                ]
                print(
                    f"[leader] seq={sequence} q_sim_deg={q_deg} "
                    f"leader_gripper_deg="
                    f"{float(action['gripper.pos']):.2f} "
                    f"gripper={payload['gripper_position']:.4f}m"
                )
            sequence += 1
            next_tick += period
            time.sleep(max(0.0, next_tick - time.perf_counter()))
    finally:
        output.close()
        if leader is not None and leader.is_connected:
            leader.disconnect()
        print("[leader] Stopped and closed the serial port")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[leader-error] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
