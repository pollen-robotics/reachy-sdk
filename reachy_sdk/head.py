"""Reachy Head module.

Handles all specific method to an Head:
- the inverse kinematics
- look_at function
"""

from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from pyquaternion import Quaternion

from .device_holder import DeviceHolder
from .joint import Joint
from .trajectory import goto, goto_async
from .trajectory.interpolation import InterpolationMode


class Head:
    """Head class.

    It exposes the neck orbita actuator at the base of the head.
    It provides look_at utility function to directly orient the head so it looks at a cartesian point
    expressed in Reachy's coordinate system.
    """

    _required_joints = (
        'neck_roll', 'neck_pitch', 'neck_yaw', 'l_antenna', 'r_antenna',
    )

    def __init__(self, joints: List[Joint], grpc_channel) -> None:
        """Set up the head."""

        def get_joint(name):
            for j in joints:
                if j.name == name:
                    return j
            raise ValueError(f'Required joint {name} not found!')

        found_joints = [get_joint(name) for name in self._required_joints]

        if len(found_joints) != len(self._required_joints):
            raise ValueError(f'Required joints not found {self._required_joints}')

        self.joints = DeviceHolder(found_joints)
        self._setup_joints(found_joints)

    @property
    def neck_orbita_joints(self):
        """Get the 3 joints composing the Orbita neck."""
        return (self.joints.neck_roll, self.joints.neck_pitch, self.joints.neck_yaw)

    def __repr__(self) -> str:
        """Clean representation of an Head state."""
        return f'<Head joints={self.joints}>'

    def _setup_joints(self, joints) -> None:
        for j in joints:
            if j.name in self._required_joints:
                setattr(self, j.name, j)

    async def look_at_async(
        self,
        x: float, y: float, z: float,
        duration: float,
        starting_positions: Optional[Dict[Joint, float]] = None,
        sampling_freq: float = 100,
        interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
    ):
        """Compute and send disks position to look at the (x, y, z) point in Reachy cartesian space.

        X is forward, Y is left and Z is upward. They all expressed in meters.

        This function is asynchronous and will return a Coroutine. This can be used to easily combined multiple goto/look_at.
        See look_at for an blocking version.

        """
        return await goto_async(
            goal_positions=self._look_at(x, y, z),
            duration=duration,
            starting_positions=starting_positions,
            sampling_freq=sampling_freq,
            interpolation_mode=interpolation_mode,
        )

    def look_at(
            self,
            x: float, y: float, z: float,
            duration: float,
            starting_positions: Optional[Dict[Joint, float]] = None,
            sampling_freq: float = 100,
            interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
    ):
        """Compute and send disks position to look at the (x, y, z) point in Reachy cartesian space.

        X is forward, Y is left and Z is upward. They all expressed in meters.

        This function will block until the movement is over. See look_at_async for an asynchronous version.

        """
        return goto(
            goal_positions=self._look_at(x, y, z),
            duration=duration,
            starting_positions=starting_positions,
            sampling_freq=sampling_freq,
            interpolation_mode=interpolation_mode,
        )

    def _look_at(self, x: float, y: float, z: float) -> Dict[Joint, float]:
        q = _find_quaternion_transform([1, 0, 0], [x, y, z])
        roll, pitch, yaw = np.rad2deg(Rotation.from_quat(q).as_euler('XYZ'))
        goal_positions = {
            self.joints.neck_roll: roll,
            self.joints.neck_pitch: pitch,
            self.joints.neck_yaw: yaw,
        }
        return goal_positions


def _find_quaternion_transform(
    vect_origin: Tuple[float, float, float],
    vect_dest: Tuple[float, float, float]
) -> Tuple[float, float, float, float]:

    vo = _norm(vect_origin)
    vd = _norm(vect_dest)

    v = np.cross(vo, vd)
    v = _norm(v)

    alpha = np.arccos(np.dot(vo, vd))
    if np.isnan(alpha) or alpha < 1e-6:
        return (0, 0, 0, 1)

    q = Quaternion(axis=v, radians=alpha)
    return (q.x, q.y, q.z, q.w)


def _norm(v):
    v = np.array(v)
    if np.any(v):
        v = v / np.linalg.norm(v)
    return v
