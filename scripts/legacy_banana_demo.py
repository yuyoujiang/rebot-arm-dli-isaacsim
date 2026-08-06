#!/usr/bin/env python3
"""Run the reBot banana grasping scene and joint-space state machine."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "grasp_config.json"
DEFAULT_SCENE = PROJECT_ROOT / "scenes" / "rebot_stationery_sorting.usda"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the GUI")
    parser.add_argument("--test", action="store_true", help="Run once and exit with success/failure status")
    parser.add_argument(
        "--strict-physics",
        action="store_true",
        help="Use only collision and friction without reliable grasp assist",
    )
    parser.add_argument(
        "--rebuild-scene", action="store_true", help="Regenerate the portable USD before running"
    )
    parser.add_argument(
        "--build-only", action="store_true", help="Build the USD scene without running control"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    return parser.parse_args()


ARGS = _parse_args()

try:
    from isaacsim import SimulationApp
except ImportError as exc:
    raise RuntimeError(
        "Run this script with /home/seeed/isaacsim/python.sh, "
        "or execute ./run.sh from the project root"
    ) from exc

simulation_app = SimulationApp({"headless": ARGS.headless or ARGS.build_only})

from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.stage import (
    get_current_stage,
    is_stage_loading,
    open_stage,
)
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdPhysics

from build_scene import build_scene


@dataclass(frozen=True)
class MotionState:
    name: str
    arm_target: np.ndarray
    gripper_target: float
    duration: float
    attach_after: bool = False


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


class BananaGraspDemo:
    def __init__(
        self,
        config_path: Path,
        scene_path: Path,
        *,
        strict_physics: bool,
    ) -> None:
        self.config_path = config_path.resolve()
        self.scene_path = scene_path.resolve()
        self.strict_physics = strict_physics
        self.config = json.loads(self.config_path.read_text(encoding="utf-8"))

        self.physics_dt = float(self.config["physics_dt"])
        self.robot_path = self.config["robot_prim_path"]
        self.banana_path = self.config["banana_prim_path"]
        self.assist_joint_path = self.config["assist_joint_path"]
        self.assist_offset = np.asarray(
            self.config["assist_grasp_offset_m"], dtype=np.float64
        )
        self.assist_orientation = np.asarray(
            self.config["assist_grasp_orientation_wxyz"], dtype=np.float64
        )
        self.arm_names = list(self.config["arm_joint_names"])
        self.gripper_names = list(self.config["gripper_joint_names"])
        self.opened = float(self.config["gripper"]["opened_m"])
        self.closed = float(self.config["gripper"]["closed_m"])
        self.initial_banana_position = np.asarray(
            self.config["banana_pose"]["position"], dtype=np.float64
        )
        self.initial_banana_orientation = np.asarray(
            self.config["banana_pose"]["orientation_wxyz"], dtype=np.float64
        )

        poses = {
            name: np.radians(np.asarray(values, dtype=np.float64))
            for name, values in self.config["poses_deg"].items()
        }
        duration = self.config["durations_s"]
        self.states = [
            MotionState("settle", poses["home"], self.opened, duration["settle"]),
            MotionState(
                "move pregrasp",
                poses["pregrasp"],
                self.opened,
                duration["move_pregrasp"],
            ),
            MotionState(
                "approach",
                poses["grasp"],
                self.opened,
                duration["approach"],
            ),
            MotionState(
                "close gripper",
                poses["grasp"],
                self.closed,
                duration["close"],
                attach_after=True,
            ),
            MotionState("lift", poses["lift"], self.closed, duration["lift"]),
            MotionState("hold and validate", poses["lift"], self.closed, duration["hold"]),
        ]

        self.world: World | None = None
        self.robot: SingleArticulation | None = None
        self.banana: SingleRigidPrim | None = None
        self.controller = None
        self.arm_indices: np.ndarray | None = None
        self.gripper_indices: np.ndarray | None = None
        self.command_arm = poses["home"].copy()
        self.command_gripper = self.opened
        self.initial_banana_z = float(self.initial_banana_position[2])

    def _wait_for_stage(self) -> None:
        while is_stage_loading():
            simulation_app.update()
        for _ in range(3):
            simulation_app.update()

    def _configure_runtime_physics(self) -> None:
        stage = get_current_stage()
        banana_prim = stage.GetPrimAtPath(self.banana_path)
        joint_prim = stage.GetPrimAtPath(self.assist_joint_path)
        if not banana_prim.IsValid():
            raise RuntimeError(f"Banana prim is missing: {self.banana_path}")
        if not joint_prim.IsValid():
            raise RuntimeError(f"Grasp-assist joint is missing: {self.assist_joint_path}")

        rigid = UsdPhysics.RigidBodyAPI(banana_prim)
        rigid.GetKinematicEnabledAttr().Set(not self.strict_physics)
        if joint_prim.IsActive():
            UsdPhysics.Joint(joint_prim).GetJointEnabledAttr().Set(False)
            joint_prim.SetActive(False)

    def setup(self) -> None:
        if not self.scene_path.exists():
            build_scene(self.scene_path)
        if not open_stage(str(self.scene_path)):
            raise RuntimeError(f"Unable to open scene: {self.scene_path}")
        self._wait_for_stage()
        self._configure_runtime_physics()

        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=self.physics_dt,
            rendering_dt=float(self.config["rendering_dt"]),
        )
        self.robot = self.world.scene.add(
            SingleArticulation(prim_path=self.robot_path, name="rebot")
        )
        # Do not register the initially kinematic banana with Scene.reset().
        # Scene.reset writes zero velocities to every registered rigid prim,
        # which PhysX correctly rejects for a kinematic body.
        self.banana = SingleRigidPrim(prim_path=self.banana_path, name="banana")
        self.world.reset()
        self.robot.initialize()
        self.banana.initialize()

        dof_names = list(self.robot.dof_names)
        missing = [
            name
            for name in self.arm_names + self.gripper_names
            if name not in dof_names
        ]
        if missing:
            raise RuntimeError(
                f"Robot DOF mismatch; missing {missing}; available DOFs: {dof_names}"
            )
        self.arm_indices = np.asarray(
            [dof_names.index(name) for name in self.arm_names], dtype=np.int64
        )
        self.gripper_indices = np.asarray(
            [dof_names.index(name) for name in self.gripper_names], dtype=np.int64
        )
        self.controller = self.robot.get_articulation_controller()

        self.robot.set_joint_positions(self.command_arm, joint_indices=self.arm_indices)
        self.robot.set_joint_velocities(
            np.zeros(len(self.arm_indices), dtype=np.float64),
            joint_indices=self.arm_indices,
        )
        self.robot.set_joint_positions(
            np.full(len(self.gripper_indices), self.opened, dtype=np.float64),
            joint_indices=self.gripper_indices,
        )
        self.robot.set_joint_velocities(
            np.zeros(len(self.gripper_indices), dtype=np.float64),
            joint_indices=self.gripper_indices,
        )
        self.banana.set_world_pose(
            position=self.initial_banana_position,
            orientation=self.initial_banana_orientation,
        )
        if self.strict_physics:
            self.banana.set_linear_velocity(np.zeros(3))
            self.banana.set_angular_velocity(np.zeros(3))
        self._apply_targets()

        if not ARGS.headless:
            camera = self.config["camera"]
            set_camera_view(
                eye=np.asarray(camera["eye"], dtype=np.float64),
                target=np.asarray(camera["target"], dtype=np.float64),
                camera_prim_path="/OmniverseKit_Persp",
            )

        for _ in range(10):
            self.world.step(render=not ARGS.headless)

        banana_position, _ = self.banana.get_world_pose()
        self.initial_banana_z = float(banana_position[2])
        mode = "strict contact physics" if self.strict_physics else "reliable grasp assist"
        print("=" * 76)
        print("  Legacy reBot DevArm banana-grasp simulation")
        print(f"  Mode: {mode}")
        print(f"  Scene: {self.scene_path}")
        print(f"  DOF : {dof_names}")
        print("=" * 76)

    def _apply_targets(self) -> None:
        assert self.controller is not None
        assert self.arm_indices is not None
        assert self.gripper_indices is not None
        joint_indices = np.concatenate((self.arm_indices, self.gripper_indices))
        joint_targets = np.concatenate(
            (
                self.command_arm,
                np.full(len(self.gripper_indices), self.command_gripper),
            )
        )
        self.controller.apply_action(
            ArticulationAction(
                joint_positions=joint_targets,
                joint_indices=joint_indices,
            )
        )

    def _enable_grasp_assist(self) -> None:
        if self.strict_physics:
            print("[grasp] Strict physics mode: no fixed grasp constraint will be created or enabled")
            return
        assert self.banana is not None

        stage = get_current_stage()
        banana_prim = stage.GetPrimAtPath(self.banana_path)
        joint_prim = stage.GetPrimAtPath(self.assist_joint_path)
        fixed_joint = UsdPhysics.FixedJoint(joint_prim)
        # These frames were calibrated against the reBot link6 grasp pose.
        # Keeping them deterministic avoids Fabric/PhysX pose-convention
        # differences when the joint is enabled at runtime.
        fixed_joint.GetLocalPos0Attr().Set(Gf.Vec3f(*self.assist_offset))
        fixed_joint.GetLocalRot0Attr().Set(
            Gf.Quatf(
                float(self.assist_orientation[0]),
                Gf.Vec3f(*self.assist_orientation[1:]),
            )
        )
        fixed_joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fixed_joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0))
        joint_prim.SetActive(True)
        rigid = UsdPhysics.RigidBodyAPI(banana_prim)
        rigid.GetKinematicEnabledAttr().Set(False)
        UsdPhysics.Joint(joint_prim).GetJointEnabledAttr().Set(True)
        print("[grasp] Gripper closed; reliable grasp constraint enabled")

    def _execute_state(self, state: MotionState) -> None:
        assert self.world is not None
        arm_start = self.command_arm.copy()
        gripper_start = self.command_gripper
        steps = max(int(round(state.duration / self.physics_dt)), 1)
        print(f"[state] {state.name} ({state.duration:.1f}s)")

        for step in range(steps):
            if not simulation_app.is_running():
                raise KeyboardInterrupt
            alpha = _smoothstep((step + 1) / steps)
            self.command_arm = arm_start + alpha * (state.arm_target - arm_start)
            self.command_gripper = (
                gripper_start + alpha * (state.gripper_target - gripper_start)
            )
            self._apply_targets()
            self.world.step(render=not ARGS.headless)

        self.command_arm = state.arm_target.copy()
        self.command_gripper = state.gripper_target
        self._apply_targets()
        if state.attach_after:
            self._enable_grasp_assist()
            for _ in range(8):
                self.world.step(render=not ARGS.headless)

    def _validate_success(self) -> bool:
        assert self.banana is not None
        position, _ = self.banana.get_world_pose()
        lifted = float(position[2]) - self.initial_banana_z
        threshold = float(self.config["success_min_lift_m"])
        success = lifted >= threshold
        result = "SUCCESS" if success else "FAILED"
        print(
            f"[{result}] banana_z={position[2]:.4f}m, "
            f"lifted={lifted:.4f}m, required={threshold:.4f}m"
        )
        return success

    def run(self) -> bool:
        assert self.world is not None
        for state in self.states:
            self._execute_state(state)
        success = self._validate_success()

        if not ARGS.test and not ARGS.headless:
            print("[done] Demo complete; close the Isaac Sim window or press Ctrl+C to exit.")
            while simulation_app.is_running():
                self._apply_targets()
                self.world.step(render=True)
        return success


def main() -> int:
    if ARGS.rebuild_scene or not ARGS.scene.exists():
        build_scene(ARGS.scene)
    if ARGS.build_only:
        print(f"[done] Scene ready: {ARGS.scene.resolve()}")
        return 0

    demo = BananaGraspDemo(
        ARGS.config,
        ARGS.scene,
        strict_physics=ARGS.strict_physics,
    )
    demo.setup()
    return 0 if demo.run() else 2


if __name__ == "__main__":
    exit_code = 1
    try:
        exit_code = main()
    except KeyboardInterrupt:
        print("\n[stop] Interrupted by user")
        exit_code = 130
    except Exception as exc:
        print(f"[error] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
    finally:
        simulation_app.close()
    raise SystemExit(exit_code)
