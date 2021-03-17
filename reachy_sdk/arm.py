from abc import ABC
from typing import List, Optional, Set

import numpy as np

from reachy_sdk_api.arm_kinematics_pb2_grpc import ArmKinematicsStub
from reachy_sdk_api.arm_kinematics_pb2 import ArmEndEffector, ArmFKRequest, ArmIKRequest, ArmJointPosition, ArmSide
from reachy_sdk_api.kinematics_pb2 import joint__pb2
from reachy_sdk_api.arm_kinematics_pb2 import kinematics__pb2

from .joint import Joint

# Circumvent https://github.com/grpc/grpc/issues/18139
JointId = joint__pb2.JointId
JointPosition = kinematics__pb2.JointPosition
Matrix4x4 = kinematics__pb2.Matrix4x4


class Arm(ABC):
    def __init__(self, joints: List[Joint], grpc_channel) -> None:
        self._kin_stub = ArmKinematicsStub(grpc_channel)

        self.joints = [j for j in joints if j.name in self._required_joints]

        if len(self.joints) != len(self._required_joints):
            raise ValueError(f'Required joints not found {self._required_joints}')

        self.kinematics_chain = [j for j in self.joints if j.name in self._kinematics_chain]

    @property
    def _side(self) -> str:
        ...

    def forward_kinematics(self, joints_position: Optional[List[float]] = None) -> np.ndarray:
        if joints_position is None:
            joints_position = [j.present_position for j in self.kinematics_chain]

        if isinstance(joints_position, np.ndarray) and len(joints_position.shape) > 1:
            raise ValueError('Vectorized kinematics not supported!')

        pos = np.deg2rad(joints_position)

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
            req_params['q0'] = self._joint_position_from_pos(q0)

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
            ids=[JointId(uid=j.uid) for j in self.kinematics_chain],
            positions=joints_position,
        )


class LeftArm(Arm):
    _side = 'left'
    _kinematics_chain = (
        'l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw',
        'l_elbow_pitch', 'l_forearm_yaw',
        'l_wrist_pitch', 'l_wrist_roll',
    )
    _required_joints = {
        'l_shoulder_pitch', 'l_shoulder_roll', 'l_arm_yaw',
        'l_elbow_pitch', 'l_forearm_yaw',
        'l_wrist_pitch', 'l_wrist_roll',
        'l_gripper',
    }


class RightArm(Arm):
    _side = 'right'
    _kinematics_chain = (
        'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw',
        'r_elbow_pitch', 'r_forearm_yaw',
        'r_wrist_pitch', 'r_wrist_roll',
    )
    _required_joints = {
        'r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw',
        'r_elbow_pitch', 'r_forearm_yaw',
        'r_wrist_pitch', 'r_wrist_roll',
        'r_gripper',
    }
