"""Reachy Head module.

Handles all specific method to an Head:
- the inverse kinematics
- look_at function
"""

from typing import Dict, List, Optional, Tuple

import numpy as np

from pyquaternion import Quaternion

from reachy_sdk_api.head_kinematics_pb2_grpc import HeadKinematicsStub
from reachy_sdk_api.head_kinematics_pb2 import HeadFKRequest, HeadIKRequest
from reachy_sdk_api.kinematics_pb2 import joint__pb2
from reachy_sdk_api.arm_kinematics_pb2 import kinematics__pb2

from .device_holder import DeviceHolder
from .joint import Joint
from .trajectory import goto, goto_async
from .trajectory.interpolation import InterpolationMode


JointId = joint__pb2.JointId
JointPosition = kinematics__pb2.JointPosition
Matrix4x4 = kinematics__pb2.Matrix4x4
QuaternionPb = kinematics__pb2.Quaternion


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
        self._kin_stub = HeadKinematicsStub(grpc_channel)

        def get_joint(name):
            for j in joints:
                if j.name == name:
                    return j
            raise ValueError(f'Required joint {name} not found!')

        found_joints = [get_joint(name) for name in self._required_joints]

        if len(found_joints) != len(self._required_joints):
            raise ValueError(f'Required joints not found {self._required_joints}')
        self._kinematics_chain = ['neck_roll', 'neck_pitch', 'neck_yaw']
        self.kinematics_chain = DeviceHolder([j for j in found_joints if j.name in self._kinematics_chain])
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

    def forward_kinematics(self, joints_position: Optional[List[float]] = None) -> Tuple[float, float, float, float]:
        """Compute the forward kinematics of the head.

        It will return the quaternion (x, y, z, w).
        You can either specify a given joints position, otherwise it will use the current robot position.
        """
        if joints_position is None:
            joints_position = [j.present_position for j in self.kinematics_chain.values()]

        if isinstance(joints_position, np.ndarray) and len(joints_position.shape) > 1:
            raise ValueError('Vectorized kinematics not supported!')

        pos = np.deg2rad(list(joints_position))

        if len(pos) != len(self._kinematics_chain):
            raise ValueError(
                f'joints_position should be length {len(self._kinematics_chain)} (got {len(pos)} instead)!'
            )

        req = HeadFKRequest(
            neck_position=self._joint_position_from_pos(pos),

        )
        resp = self._kin_stub.ComputeHeadFK(req)
        if not resp.success:
            raise ValueError(f'No solution found for the given joints ({joints_position})!')

        return (resp.q.x, resp.q.y, resp.q.z, resp.q.w)

    def inverse_kinematics(self, target: Tuple[float, float, float, float], q0: Optional[List[float]] = None) -> List[float]:
        """Compute the inverse kinematics of the arm.

        Given a goal quaternion (x, y, z, w)
        it will try to compute a joint solution to reach this target (or get close).

        It will raise a ValueError if no solution is found.

        You can also specify a basic joint configuration as a prior for the solution.
        """
        if q0 is not None and (len(q0) != len(self._kinematics_chain)):
            raise ValueError(f'q0 should be length {len(self._kinematics_chain)} (got {len(q0)} instead)!')

        if isinstance(q0, np.ndarray) and len(q0.shape) > 1:
            raise ValueError('Vectorized kinematics not supported!')

        qr = QuaternionPb()
        qr.x = target[0]
        qr.y = target[1]
        qr.z = target[2]
        qr.w = target[3]

        req_params = {
            'q': qr
        }

        if q0 is not None:
            req_params['q0'] = self._joint_position_from_pos(np.deg2rad(q0))

        req = HeadIKRequest(**req_params)
        resp = self._kin_stub.ComputeHeadIK(req)

        if not resp.success:
            raise ValueError(f'No solution found for the given target ({target})!')

        return np.rad2deg(resp.neck_position.positions).tolist()

    def _joint_position_from_pos(self, joints_position: List[float]) -> JointPosition:
        return JointPosition(
            ids=[JointId(uid=j.uid) for j in self.kinematics_chain.values()],
            positions=joints_position,
        )

    async def look_at_async(
        self,
        x: float, y: float, z: float,
        duration: float,
        starting_positions: Optional[Dict[Joint, float]] = None,
        sampling_freq: float = 100,
        interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
    ):
        """Compute and send neck rpy position to look at the (x, y, z) point in Reachy cartesian space (torso frame).

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
        """Compute and send neck rpy position to look at the (x, y, z) point in Reachy cartesian space (torso frame).

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

        neckik = self.inverse_kinematics(q)
        roll, pitch, yaw = neckik[0], neckik[1], neckik[2]
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

    # ad-hoc translation to move in the torso frame (from urdf)
    orbita_in_torso = (vect_dest[0]-0.015, vect_dest[1], vect_dest[2]-0.095)
    # simple approximation, hopefully good enough...
    head_in_torso = (orbita_in_torso[0]-0.02, orbita_in_torso[1], orbita_in_torso[2]-0.06105)

    vd = _norm(head_in_torso)

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
