"""
urdf_kinematics.py — 纯 Python URDF 正运动学

从 b29_locomotion/sim2sim_mujoco/gp11_reach_runtime.py 提取，
仅保留 UrdfKinematicModel 及其依赖的数学辅助函数。
零外部依赖（只需 numpy 和标准库），可在任意部署环境使用。
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple, Union

import numpy as np


# --------------------------------------------------------------------------- #
# 数学辅助函数
# --------------------------------------------------------------------------- #

def _require_vector(name: str, values: np.ndarray, expected_size: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32)
    if array.shape != (expected_size,):
        raise ValueError(f"{name} must have shape ({expected_size},), got {array.shape}")
    return array


def _normalize_vector(values: np.ndarray, eps: float = 1.0e-6) -> np.ndarray:
    array = np.asarray(values, dtype=np.float32)
    norm = float(np.linalg.norm(array))
    if norm < eps:
        raise ValueError("vector norm is too small to normalize")
    return (array / norm).astype(np.float32, copy=False)


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1., 0., 0.], [0., cr, -sr], [0., sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0., sp], [0., 1., 0.], [-sp, 0., cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0.], [sy, cy, 0.], [0., 0., 1.]], dtype=np.float32)
    return (rz @ ry @ rx).astype(np.float32, copy=False)


def _axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = _normalize_vector(axis)
    x, y, z = [float(v) for v in axis]
    c, s = np.cos(angle), np.sin(angle)
    omc = 1.0 - c
    return np.array([
        [c + x*x*omc,     x*y*omc - z*s, x*z*omc + y*s],
        [y*x*omc + z*s,   c + y*y*omc,   y*z*omc - x*s],
        [z*x*omc - y*s,   z*y*omc + x*s, c + z*z*omc  ],
    ], dtype=np.float32)


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=np.float32)
    t[:3, :3] = np.asarray(rotation, dtype=np.float32)
    t[:3, 3]  = np.asarray(translation, dtype=np.float32)
    return t


# --------------------------------------------------------------------------- #
# URDF 关节描述
# --------------------------------------------------------------------------- #

@dataclass
class _UrdfJointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray


# --------------------------------------------------------------------------- #
# 正运动学模型
# --------------------------------------------------------------------------- #

class UrdfKinematicModel:
    """纯 Python URDF 正运动学，用于计算末端位置和 obs_ref 坐标系。

    与 b29_locomotion/sim2sim_mujoco/gp11_reach_runtime.py 中的同名类接口一致。
    """

    ACTIVE_JOINTS = (
        "left_first_leg_joint",
        "left_second_leg_joint",
        "right_first_leg_joint",
        "right_second_leg_joint",
    )

    def __init__(
        self,
        *,
        root_link: str,
        joints: Sequence[_UrdfJointSpec],
        tool_left_body: str,
        tool_right_body: str,
        orientation_body: str,
    ) -> None:
        self.root_link = root_link
        self._joints = tuple(joints)
        self._children_by_parent: Dict[str, List[_UrdfJointSpec]] = {}
        self._links = {root_link}
        for joint in self._joints:
            self._children_by_parent.setdefault(joint.parent, []).append(joint)
            self._links.add(joint.parent)
            self._links.add(joint.child)
        self.tool_left_body  = tool_left_body
        self.tool_right_body = tool_right_body
        self.orientation_body = orientation_body
        self._joint_names = {joint.name for joint in self._joints}

        missing_links = [
            n for n in (tool_left_body, tool_right_body, orientation_body)
            if n not in self._links
        ]
        if missing_links:
            raise ValueError(f"URDF missing required links: {missing_links}")

        missing_joints = [n for n in self.ACTIVE_JOINTS if n not in self._joint_names]
        if missing_joints:
            raise ValueError(f"URDF missing required joints: {missing_joints}")

        reachable = self._compute_link_transforms(np.zeros(4, dtype=np.float32))
        unreachable = [
            n for n in (tool_left_body, tool_right_body, orientation_body)
            if n not in reachable
        ]
        if unreachable:
            raise ValueError(f"Links not reachable from root '{root_link}': {unreachable}")

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Union[Path, str],
        *,
        tool_left_body: str,
        tool_right_body: str,
        orientation_body: str,
    ) -> "UrdfKinematicModel":
        root = ET.parse(Path(urdf_path)).getroot()
        links = {l.get("name") for l in root.findall("link") if l.get("name")}
        child_links: set = set()
        joints: List[_UrdfJointSpec] = []
        for jel in root.findall("joint"):
            name = jel.get("name")
            if not name:
                continue
            parent_el = jel.find("parent")
            child_el  = jel.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.get("link")
            child  = child_el.get("link")
            if not parent or not child:
                continue
            child_links.add(child)
            origin_el = jel.find("origin")
            axis_el   = jel.find("axis")
            xyz = np.zeros(3, dtype=np.float32)
            rpy = np.zeros(3, dtype=np.float32)
            axis = np.array([0., 0., 1.], dtype=np.float32)
            if origin_el is not None:
                xyz  = np.array([float(v) for v in origin_el.get("xyz", "0 0 0").split()], dtype=np.float32)
                rpy  = np.array([float(v) for v in origin_el.get("rpy", "0 0 0").split()], dtype=np.float32)
            if axis_el is not None:
                axis = np.array([float(v) for v in axis_el.get("xyz", "0 0 1").split()], dtype=np.float32)
            joints.append(_UrdfJointSpec(
                name=name, joint_type=jel.get("type", "fixed"),
                parent=parent, child=child,
                origin_xyz=xyz, origin_rpy=rpy, axis=axis,
            ))
        root_links = sorted(links - child_links)
        if len(root_links) != 1:
            raise ValueError(f"expected one root link, found {root_links}")
        return cls(
            root_link=root_links[0], joints=joints,
            tool_left_body=tool_left_body, tool_right_body=tool_right_body,
            orientation_body=orientation_body,
        )

    def _compute_link_transforms(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        q = _require_vector("q", q, 4)
        joint_values = {name: float(val) for name, val in zip(self.ACTIVE_JOINTS, q)}
        transforms: Dict[str, np.ndarray] = {self.root_link: np.eye(4, dtype=np.float32)}
        stack = [self.root_link]
        while stack:
            parent = stack.pop()
            T_parent = transforms[parent]
            for joint in self._children_by_parent.get(parent, []):
                T_origin = _make_transform(_rpy_to_matrix(joint.origin_rpy), joint.origin_xyz)
                angle = joint_values.get(joint.name, 0.0)
                R_joint = np.eye(3, dtype=np.float32) if joint.joint_type == "fixed" \
                    else _axis_angle_to_matrix(joint.axis, angle)
                T_child = T_parent @ T_origin @ _make_transform(R_joint, np.zeros(3, dtype=np.float32))
                transforms[joint.child] = T_child.astype(np.float32, copy=False)
                stack.append(joint.child)
        return transforms

    def evaluate(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        transforms = self._compute_link_transforms(q)
        tool_l = transforms[self.tool_left_body][:3, 3]
        tool_r = transforms[self.tool_right_body][:3, 3]
        midpoint = (0.5 * (tool_l + tool_r)).astype(np.float32, copy=False)
        x_axis   = _normalize_vector(transforms[self.orientation_body][:3, 0])
        return midpoint, x_axis

    def relative_x_axis(
        self,
        q: np.ndarray,
        *,
        reference_body: str,
        axis_body: Optional[str] = None,
    ) -> np.ndarray:
        """返回 axis_body 的 x 轴在 reference_body 坐标系下的表达。"""
        transforms = self._compute_link_transforms(q)
        resolved_axis_body = self.orientation_body if axis_body is None else axis_body
        missing = [
            name for name in (reference_body, resolved_axis_body)
            if name not in transforms
        ]
        if missing:
            raise ValueError(f"URDF is missing relative-axis links: {missing}")
        reference_rotation = transforms[reference_body][:3, :3]
        axis_root = transforms[resolved_axis_body][:3, 0]
        return _normalize_vector(reference_rotation.T @ axis_root)

    def nominal_link_transform(self, link_name: str) -> np.ndarray:
        transforms = self._compute_link_transforms(np.zeros(4, dtype=np.float32))
        if link_name not in transforms:
            raise ValueError(f"URDF missing link '{link_name}'")
        return transforms[link_name].astype(np.float32, copy=True)
