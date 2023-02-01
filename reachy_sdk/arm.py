"""Reachy Arm module.

Handles all specific method to an Arm (left and/or right) especially:
- the forward kinematics
- the inverse kinematics
"""

from abc import ABC
from typing import List, Optional, Set

import numpy as np

from reachy_sdk_api.arm_kinematics_pb2_grpc import ArmKinematicsStub
from reachy_sdk_api.arm_kinematics_pb2 import ArmEndEffector, ArmFKRequest, ArmIKRequest, ArmJointPosition, ArmSide
from reachy_sdk_api.kinematics_pb2 import joint__pb2
from reachy_sdk_api.arm_kinematics_pb2 import kinematics__pb2

from .device_holder import DeviceHolder
from .joint import Joint

# Circumvent https://github.com/grpc/grpc/issues/18139
JointId = joint__pb2.JointId
JointPosition = kinematics__pb2.JointPosition
Matrix4x4 = kinematics__pb2.Matrix4x4


class Arm(ABC):
    """Arm abstract class used for both left/right arms.

    It exposes the kinematics of the arm:
    - you can access the joints actually used in the kinematic chain,
    - you can compute the forward and inverse kinematics
    """

    def __init__(self, joints: List[Joint], grpc_channel) -> None:
        """Set up the arm with its kinematics."""
        self._kin_stub = ArmKinematicsStub(grpc_channel)

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

        self.kinematics_chain = DeviceHolder([j for j in found_joints if j.name in self._kinematics_chain])

    def __repr__(self) -> str:
        """Clean representation of an arm state."""
        return f'<Arm side="{self._side}" joints={self.joints}>'

    @property
    def _side(self) -> str:
        ...

    def _setup_joints(self, joints: List[Joint]) -> None:
        for j in joints:
            if j.name in self._required_joints:
                setattr(self, j.name, j)

    def forward_kinematics(self, joints_position: Optional[List[float]] = None) -> np.ndarray:
        """Compute the forward kinematics of the arm.

        It will return the pose 4x4 matrix (as a numpy array) expressed in Reachy coordinate systems.
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

        req = ArmFKRequest(
            arm_position=ArmJointPosition(
                side=self._arm_side,
                positions=self._joint_position_from_pos(pos),
            ),
        )
        resp = self._kin_stub.ComputeArmFK(req)
        if not resp.success:
            raise ValueError(f'No solution found for the given joints ({joints_position})!')

        return np.array(resp.end_effector.pose.data).reshape((4, 4))

    def inverse_kinematics(self, target: np.ndarray, q0: Optional[List[float]] = None) -> List[float]:
        """Compute the inverse kinematics of the arm.

        Given a pose 4x4 target matrix (as a numpy array) expressed in Reachy coordinate systems,
        it will try to compute a joint solution to reach this target (or get close).

        It will raise a ValueError if no solution is found.

        You can also specify a basic joint configuration as a prior for the solution.
        """
        if target.shape != (4, 4):
            raise ValueError('target shape should be (4, 4) (got {target.shape} instead)!')

        if q0 is not None and (len(q0) != len(self._kinematics_chain)):
            raise ValueError(f'q0 should be length {len(self._kinematics_chain)} (got {len(q0)} instead)!')

        if isinstance(q0, np.ndarray) and len(q0.shape) > 1:
            raise ValueError('Vectorized kinematics not supported!')

        req_params = {
            'target': ArmEndEffector(
                side=self._arm_side,
                pose=Matrix4x4(data=target.flatten().tolist()),
            )
        }

        if q0 is not None:
            req_params['q0'] = self._joint_position_from_pos(np.deg2rad(q0))

        req = ArmIKRequest(**req_params)
        resp = self._kin_stub.ComputeArmIK(req)

        if not resp.success:
            raise ValueError(f'No solution found for the given target ({target})!')

        return np.rad2deg(resp.arm_position.positions.positions).tolist()

    @property
    def _kinematics_chain(self) -> List[str]:
        ...

    @property
    def _required_joints(self) -> Set[str]:
        ...

    @property
    def _arm_side(self):
        return ArmSide.LEFT if self._side == 'left' else ArmSide.RIGHT

    def _joint_position_from_pos(self, joints_position: List[float]) -> ArmJointPosition:
        return JointPosition(
            ids=[JointId(uid=j.uid) for j in self.kinematics_chain.values()],
            positions=joints_position,
        )


class LeftArm(Arm):
    """LeftArm class, all the work is actually done by the ABC Arm class.

    It exposes the kinematics of the arm:
    - you can access the joints actually used in the kinematic chain,
    - you can compute the forward and inverse kinematics
    """

    _side = 'left'
    _kinematics_chain = (
        'l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw',
        'l_elbow_pitch', 'l_forearm_yaw',
        'l_wrist_pitch', 'l_wrist_roll',
    )
    _required_joints = (
        'l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw',
        'l_elbow_pitch', 'l_forearm_yaw',
        'l_wrist_pitch', 'l_wrist_roll',
        'l_gripper',
    )


class RightArm(Arm):
    """RightArm class, all the work is actually done by the ABC Arm class.

    It exposes the kinematics of the arm:
    - you can access the joints actually used in the kinematic chain,
    - you can compute the forward and inverse kinematics
    """

    _side = 'right'
    _kinematics_chain = (
        'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw',
        'r_elbow_pitch', 'r_forearm_yaw',
        'r_wrist_pitch', 'r_wrist_roll',
    )
    _required_joints = (
        'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw',
        'r_elbow_pitch', 'r_forearm_yaw',
        'r_wrist_pitch', 'r_wrist_roll',
        'r_gripper',
    )
