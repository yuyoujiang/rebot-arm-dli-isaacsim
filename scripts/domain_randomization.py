#!/usr/bin/env python3
"""Reset-time randomization for the reBot stationery-sorting scene."""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics


DOME_LIGHT = "/World/Lights/Dome"
KEY_LIGHT = "/World/Lights/Key"


def _sample(rng: np.random.Generator, bounds: list[float]) -> float:
    return float(rng.uniform(float(bounds[0]), float(bounds[1])))


def _sample_rgb(
    rng: np.random.Generator, ranges: dict[str, list[float]]
) -> tuple[float, float, float]:
    return tuple(_sample(rng, ranges[channel]) for channel in ("r", "g", "b"))


def _jitter_vector(
    rng: np.random.Generator,
    base: list[float],
    ranges: dict[str, list[float]],
) -> np.ndarray:
    return np.asarray(base, dtype=np.float64) + np.asarray(
        [_sample(rng, ranges[axis]) for axis in ("x", "y", "z")],
        dtype=np.float64,
    )


class DomainRandomizer:
    """Apply deterministic, reset-indexed object and visual variation."""

    def __init__(
        self,
        *,
        stage: Usd.Stage,
        targets: dict[str, Any],
        side_camera,
        front_camera,
        config: dict[str, Any],
        enabled: bool,
        seed: int,
    ) -> None:
        self.stage = stage
        self.targets = targets
        self.side_camera = side_camera
        self.front_camera = front_camera
        self.config = config
        self.enabled = enabled
        self.seed = int(seed)
        self.reset_index = 0

    def _set_light(
        self,
        prim_path: str,
        intensity: float,
        color: tuple[float, float, float],
    ) -> None:
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Light prim is missing: {prim_path}")
        prim.GetAttribute("inputs:intensity").Set(float(intensity))
        prim.GetAttribute("inputs:color").Set(Gf.Vec3f(*color))

    @staticmethod
    def _set_camera_look_at(
        camera,
        *,
        eye: np.ndarray,
        target: np.ndarray,
        up: np.ndarray,
        focal_length: float,
    ) -> None:
        eye_gf = Gf.Vec3d(*np.asarray(eye, dtype=np.float64))
        target_gf = Gf.Vec3d(*np.asarray(target, dtype=np.float64))
        up_gf = Gf.Vec3d(*np.asarray(up, dtype=np.float64))
        matrix = Gf.Matrix4d().SetLookAt(
            eye_gf, target_gf, up_gf
        ).GetInverse()
        xform = UsdGeom.Xformable(camera.prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(matrix)
        camera.prim.GetAttribute("focalLength").Set(float(focal_length))
        camera.prim.GetAttribute("focusDistance").Set(
            float(np.linalg.norm(target - eye))
        )

    def reset(self) -> dict[str, Any]:
        rng = np.random.default_rng(self.seed + self.reset_index)
        reset_seed = self.seed + self.reset_index
        self.reset_index += 1

        object_results: dict[str, dict[str, Any]] = {}
        for name, target in self.targets.items():
            object_cfg = self.config["objects"][name]
            position = np.asarray(
                object_cfg["base_position_m"], dtype=np.float64
            )
            base_yaw = float(object_cfg.get("base_yaw_deg", 0.0))
            if self.enabled:
                position[0] += _sample(rng, object_cfg["x_offset_m"])
                position[1] += _sample(rng, object_cfg["y_offset_m"])
                yaw_deg = base_yaw + _sample(rng, object_cfg["yaw_jitter_deg"])
                mass = _sample(rng, object_cfg["mass_kg"])
            else:
                yaw_deg = base_yaw
                mass = float(sum(object_cfg["mass_kg"]) / 2.0)
            yaw = math.radians(yaw_deg)
            orientation = np.asarray(
                [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
                dtype=np.float64,
            )
            target.set_world_pose(position=position, orientation=orientation)
            target.set_linear_velocity(np.zeros(3, dtype=np.float64))
            target.set_angular_velocity(np.zeros(3, dtype=np.float64))
            UsdPhysics.MassAPI(target.prim).GetMassAttr().Set(float(mass))
            object_results[name] = {
                "position_m": position.tolist(),
                "yaw_deg": yaw_deg,
                "mass_kg": mass,
            }

        lighting_cfg = self.config["lighting"]
        if self.enabled:
            dome_intensity = _sample(rng, lighting_cfg["dome_intensity"])
            key_intensity = _sample(rng, lighting_cfg["key_intensity"])
            dome_color = _sample_rgb(rng, lighting_cfg["dome_color_rgb"])
            key_color = _sample_rgb(rng, lighting_cfg["key_color_rgb"])
        else:
            dome_intensity = 800.0
            key_intensity = 650.0
            dome_color = (0.94, 0.96, 1.0)
            key_color = (1.0, 0.94, 0.82)
        self._set_light(DOME_LIGHT, dome_intensity, dome_color)
        self._set_light(KEY_LIGHT, key_intensity, key_color)

        camera_results: dict[str, dict[str, Any]] = {}
        for name, camera in (
            ("side_camera", self.side_camera),
            ("front_camera", self.front_camera),
        ):
            camera_cfg = self.config[name]
            if self.enabled:
                eye = _jitter_vector(
                    rng,
                    camera_cfg["base_eye_m"],
                    camera_cfg["position_jitter_m"],
                )
                target = _jitter_vector(
                    rng,
                    camera_cfg["base_target_m"],
                    camera_cfg["target_jitter_m"],
                )
                focal = _sample(rng, camera_cfg["focal_length_mm"])
            else:
                eye = np.asarray(camera_cfg["base_eye_m"], dtype=np.float64)
                target = np.asarray(
                    camera_cfg["base_target_m"], dtype=np.float64
                )
                focal = float(sum(camera_cfg["focal_length_mm"]) / 2.0)
            up = np.asarray(
                camera_cfg.get("up_m", [0.0, 0.0, 1.0]),
                dtype=np.float64,
            )
            self._set_camera_look_at(
                camera, eye=eye, target=target, up=up, focal_length=focal
            )
            if "clipping_range_m" in camera_cfg:
                camera.prim.GetAttribute("clippingRange").Set(
                    Gf.Vec2f(*camera_cfg["clipping_range_m"])
                )
            camera_results[name] = {
                "eye_m": eye.tolist(),
                "target_m": target.tolist(),
                "focal_length_mm": focal,
            }

        return {
            "enabled": self.enabled,
            "seed": reset_seed,
            "reset_index": self.reset_index - 1,
            "objects": object_results,
            "lighting": {
                "dome_intensity": dome_intensity,
                "key_intensity": key_intensity,
                "dome_color_rgb": list(dome_color),
                "key_color_rgb": list(key_color),
            },
            **camera_results,
        }
