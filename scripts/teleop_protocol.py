#!/usr/bin/env python3
"""Pure helpers for the reBot leader-to-Isaac UDP protocol."""

from __future__ import annotations

import json
import math
from typing import Any


ARM_JOINT_COUNT = 6


def _finite_float(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


def leader_action_to_payload(
    action: dict[str, Any],
    config: dict[str, Any],
    *,
    sequence: int,
    timestamp: float,
) -> dict[str, Any]:
    """Convert LeRobot leader values in degrees to Seeed's Isaac UDP payload."""
    joint_positions: list[float] = []
    for mapping in config["arm_mapping"]:
        leader_key = f"{mapping['leader_joint']}.pos"
        if leader_key not in action:
            raise KeyError(f"Leader data is missing {leader_key}")
        leader_deg = _finite_float(action[leader_key], leader_key)
        sim_deg = (
            leader_deg * _finite_float(mapping["scale"], f"{leader_key}.scale")
            + _finite_float(mapping.get("offset_deg", 0.0), f"{leader_key}.offset")
        )
        joint_positions.append(math.radians(sim_deg))

    if len(joint_positions) != ARM_JOINT_COUNT:
        raise ValueError(
            f"arm_mapping must contain {ARM_JOINT_COUNT} joints; "
            f"received {len(joint_positions)}"
        )

    gripper = config["gripper_mapping"]
    gripper_key = f"{gripper['leader_joint']}.pos"
    if gripper_key not in action:
        raise KeyError(f"Leader data is missing {gripper_key}")
    leader_gripper = _finite_float(action[gripper_key], gripper_key)
    leader_closed = _finite_float(
        gripper["leader_closed_deg"], "leader_closed_deg"
    )
    leader_open = _finite_float(gripper["leader_open_deg"], "leader_open_deg")
    span = leader_open - leader_closed
    if abs(span) < 1e-9:
        raise ValueError("leader_open_deg cannot equal leader_closed_deg")
    openness = min(1.0, max(0.0, (leader_gripper - leader_closed) / span))
    sim_closed = _finite_float(gripper["sim_closed_m"], "sim_closed_m")
    sim_open = _finite_float(gripper["sim_open_m"], "sim_open_m")
    gripper_position = sim_closed + openness * (sim_open - sim_closed)

    return {
        "sequence": int(sequence),
        "timestamp": _finite_float(timestamp, "timestamp"),
        "joint_positions": joint_positions,
        "gripper_position": gripper_position,
        "source": "rebot_arm_102_leader",
    }


def encode_payload(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def decode_payload(packet: bytes) -> tuple[list[float], float, int, float]:
    """Validate a Seeed-compatible packet and return q, gripper, seq, timestamp."""
    payload = json.loads(packet.decode("utf-8"))
    sequence = int(payload["sequence"])
    timestamp = _finite_float(payload["timestamp"], "timestamp")
    joint_positions = [
        _finite_float(value, f"joint_positions[{index}]")
        for index, value in enumerate(payload["joint_positions"])
    ]
    if len(joint_positions) != ARM_JOINT_COUNT:
        raise ValueError(
            f"joint_positions has length {len(joint_positions)}; "
            f"expected {ARM_JOINT_COUNT}"
        )
    gripper_position = _finite_float(
        payload.get("gripper_position", 0.0), "gripper_position"
    )
    return joint_positions, gripper_position, sequence, timestamp
