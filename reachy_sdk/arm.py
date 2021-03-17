from abc import ABC
from typing import List, Optional, Set

import numpy as np

from reachy_sdk_api.arm_kinematics_pb2_grpc import ArmKinematicsStub
from reachy_sdk_api.arm_kinematics_pb2 import ArmFKRequest, ArmJointPosition, ArmSide
from reachy_sdk_api.kinematics_pb2 import JointPosition
from reachy_sdk_api.joint_pb2 import JointId

from .joint import Joint


class Arm(ABC):
    def __init__(self, joints: List[Joint], grpc_channel) -> None:
        self._kin_stub = ArmKinematicsStub(grpc_channel)

        self.joints = [j for j in joints if j.name in self._required_joints]

        if len(self.joints) != len(self._required_joints):
            raise ValueError(f'Required joints not found {self._required_joints}')

        self._joint_ids = [JointId(uid=j.uid) for j in self.joints]

    @property
    def side(self) -> str:
        ...

    def forward_kinematics(self, joints_position: Optional[List[float]] = None, use_rad: bool = False) -> np.ndarray:
        if joints_position is None:
            joints_position = [j.present_position for j in self.joints]

        if not use_rad:
            joints_position = np.deg2rad(joints_position)

        req = ArmFKRequest(
            arm_position=ArmJointPosition(
                side=ArmSide.LEFT if self.side == 'left' else ArmSide.RIGHT,
                positions=JointPosition(
                    ids=self._joint_ids,
                    positions=joints_position,
                ),
            ),
        )
        resp = self._kin_stub.ComputeArmFK(req)
        if not resp.success:
            raise ValueError('No solution found for the given joints!')

        return np.array(resp.end_effector.pose.data).reshape((4, 4))

    def inverse_kinematics(self):
        pass

    @property
    def _required_joints(self) -> Set[str]:
        ...



class LeftArm(Arm):
    pass


class RightArm(Arm):
    pass
