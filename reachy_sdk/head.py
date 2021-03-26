"""Reachy Head module.

Handles all specific method to an Head:
- the inverse kinematics
- look_at function
"""

from typing import Dict, List, Optional

import numpy as np

from reachy_sdk_api.joint_pb2 import JointId
from reachy_sdk_api.orbita_kinematics_pb2_grpc import OrbitaKinematicsStub
from reachy_sdk_api.orbita_kinematics_pb2 import LookVector, OrbitaIKRequest, kinematics__pb2

from .joint import Joint
from .trajectory import goto, goto_async
from .trajectory.interpolation import InterpolationMode


class Head:
    """Head class.

    It exposes the inverse kinematics of the head.
    It provides look_at utility function to directly orient the head so it looks at a cartesian point
    expressed in Reachy's coordinate system.
    """

    _required_joints = {
        'neck_disk_bottom', 'neck_disk_middle', 'neck_disk_top',
    }

    def __init__(self, joints: List[Joint], grpc_channel) -> None:
        """Set up the head with its kinematics."""
        self.joints = [j for j in joints if j.name in self._required_joints]

        if len(self.joints) != len(self._required_joints):
            raise ValueError(f'Required joints not found {self._required_joints}')

        self._stub = OrbitaKinematicsStub(grpc_channel)

    @property
    def disks(self):
        """Return the three orbita disks. The inverse kinematics will always return the disk position in the same order."""
        return self._required_joints

    def inverse_kinematics(self, quaternion: np.ndarray) -> List[float]:
        """Compute the inverse kinematics of the head.

        Given a target quaternion (where X is forward, Y is left and Z is upward),
        it will try to compute disk positions solution to reach this target.

        It will raise a ValueError if no solution is found.
        """
        quaternion = np.array(quaternion)
        if quaternion.shape != (4, ):
            raise ValueError('Quaternion should be given as np.array([x, y, z, w])!')
        x, y, z, w = quaternion

        req = OrbitaIKRequest(
            q=kinematics__pb2.Quaternion(w=w, x=x, y=y, z=z),
        )

        solution = self._stub.ComputeOrbitaIK(req)
        if not solution.success:
            raise ValueError(f'Could not find a solution for the given quaternion {quaternion}!')

        d = {
            self._get_joint_from_id(joint_id): np.rad2deg(pos)

            for joint_id, pos in zip(
                solution.disk_position.ids,
                solution.disk_position.positions,
            )
        }
        return [d[j] for j in self.joints]

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
        q = self._stub.GetQuaternionTransform(LookVector(x=x, y=y, z=z))
        goal_positions = self.inverse_kinematics(np.array((q.x, q.y, q.z, q.w)))
        return dict(zip(self.joints, goal_positions))

    def _get_joint_from_id(self, joint_id: JointId) -> Joint:
        if joint_id.HasField('uid'):
            for j in self.joints:
                if j.uid == joint_id.uid:
                    return j
            else:
                raise IndexError(f'No joints found for Id {joint_id}')
        else:
            return getattr(self, joint_id.name)
