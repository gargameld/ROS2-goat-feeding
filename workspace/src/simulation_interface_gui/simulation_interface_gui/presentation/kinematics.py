"""Pure mathematical helpers for the robot's top-view representation."""

from dataclasses import dataclass
import math
from typing import Sequence

from simulation_interface_gui.models import Point3D
from simulation_interface_gui.models import Quaternion


_Rotation = tuple[float, float, float, float, float, float, float, float, float]
_IDENTITY_ROTATION: _Rotation = (
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0,
)


@dataclass(frozen=True, slots=True)
class _Transform3D:
    """Store a small rigid transform without requiring a matrix library."""

    rotation: _Rotation = _IDENTITY_ROTATION
    translation: Point3D = Point3D(0.0, 0.0, 0.0)

    def then(self, child: '_Transform3D') -> '_Transform3D':
        """Compose this parent transform with ``child``."""
        return _Transform3D(
            rotation=_multiply_rotations(self.rotation, child.rotation),
            translation=_add_points(
                _rotate_point(self.rotation, child.translation),
                self.translation,
            ),
        )


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    """Return planar yaw from a possibly non-unit quaternion."""
    norm_squared = (
        quaternion.w * quaternion.w
        + quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    if norm_squared <= 1e-24:
        raise ValueError('Cannot calculate yaw from a zero quaternion.')

    scale = 1.0 / math.sqrt(norm_squared)
    w = quaternion.w * scale
    x = quaternion.x * scale
    y = quaternion.y * scale
    z = quaternion.z * scale
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def robot_point_to_world(
    point: Point3D,
    base_position: Point3D,
    base_yaw: float,
) -> Point3D:
    """Rotate a chassis-relative point by yaw and translate it into the world."""
    cosine = math.cos(base_yaw)
    sine = math.sin(base_yaw)
    return Point3D(
        x=base_position.x + cosine * point.x - sine * point.y,
        y=base_position.y + sine * point.x + cosine * point.y,
        z=base_position.z + point.z,
    )


class ArmKinematics:
    """Calculate six arm-joint origins and the tool point from MJCF geometry."""

    joint_count = 6

    def forward(self, joint_positions: Sequence[float]) -> tuple[Point3D, ...]:
        """
        Return chassis-relative joint and tool positions.

        The fixed offsets and rotations match ``mujoco_model/mounted_arm.xml``. The
        returned tuple contains the six actuated joint origins followed by the
        gripper attachment point.
        """
        if len(joint_positions) != self.joint_count:
            raise ValueError(
                f'Expected {self.joint_count} arm joints, '
                f'got {len(joint_positions)}.'
            )
        joints = tuple(float(value) for value in joint_positions)

        arm_base = (
            _translation(0.0, 0.0, 0.10)
            .then(_translation(0.0, 0.0, 0.09))
            .then(_rotation_z(math.pi))
        )
        shoulder = (
            arm_base
            .then(_translation(0.0, 0.0, 0.163))
            .then(_rotation_z(joints[0]))
        )
        upper_arm = (
            shoulder
            .then(_translation(0.0, 0.138, 0.0))
            .then(_rotation_y(math.pi / 2.0))
            .then(_rotation_y(joints[1]))
        )
        forearm = (
            upper_arm
            .then(_translation(0.0, -0.131, 0.425))
            .then(_rotation_y(joints[2]))
        )
        wrist_1 = (
            forearm
            .then(_translation(0.0, 0.0, 0.392))
            .then(_rotation_y(math.pi / 2.0))
            .then(_rotation_y(joints[3]))
        )
        wrist_2 = (
            wrist_1
            .then(_translation(0.0, 0.127, 0.0))
            .then(_rotation_z(joints[4]))
        )
        wrist_3 = (
            wrist_2
            .then(_translation(0.0, 0.0, 0.1))
            .then(_rotation_y(joints[5]))
        )
        tool = wrist_3.then(_translation(0.0, 0.1, 0.0))

        return tuple(
            transform.translation
            for transform in (
                shoulder,
                upper_arm,
                forearm,
                wrist_1,
                wrist_2,
                wrist_3,
                tool,
            )
        )


def _translation(x: float, y: float, z: float) -> _Transform3D:
    return _Transform3D(translation=Point3D(x, y, z))


def _rotation_y(angle: float) -> _Transform3D:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return _Transform3D(rotation=(
        cosine, 0.0, sine,
        0.0, 1.0, 0.0,
        -sine, 0.0, cosine,
    ))


def _rotation_z(angle: float) -> _Transform3D:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return _Transform3D(rotation=(
        cosine, -sine, 0.0,
        sine, cosine, 0.0,
        0.0, 0.0, 1.0,
    ))


def _add_points(first: Point3D, second: Point3D) -> Point3D:
    return Point3D(first.x + second.x, first.y + second.y, first.z + second.z)


def _rotate_point(rotation: _Rotation, point: Point3D) -> Point3D:
    return Point3D(
        rotation[0] * point.x + rotation[1] * point.y + rotation[2] * point.z,
        rotation[3] * point.x + rotation[4] * point.y + rotation[5] * point.z,
        rotation[6] * point.x + rotation[7] * point.y + rotation[8] * point.z,
    )


def _multiply_rotations(first: _Rotation, second: _Rotation) -> _Rotation:
    return tuple(
        sum(first[row * 3 + offset] * second[offset * 3 + column]
            for offset in range(3))
        for row in range(3)
        for column in range(3)
    )
