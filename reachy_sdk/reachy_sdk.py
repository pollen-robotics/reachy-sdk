"""ReachySDK package.

This package provides remote access (via socket) to a Reachy robot.
It automatically handles the synchronization with the robot.
In particular, you can easily get an always up-to-date robot state (joint positions, sensors value).
You can also send joint commands, compute forward or inverse kinematics.

"""

import asyncio
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional

import numpy as np

import grpc
from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api import camera_reachy_pb2_grpc
from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import fan_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc

from .arm import LeftArm, RightArm
from .camera import Camera
from .fan import Fan
from .force_sensor import ForceSensor
from .joint import Joint
from .trajectory.interpolation import InterpolationMode


class ReachySDK:
    """The ReachySDK class handles the connection with your robot.

    It holds:

    - all joints (can be accessed directly via their name or via the joints list).
    - all force sensors (can be accessed directly via their name or via the force_sensors list).
    - all fans (can be accessed directly via their name or via the fans list).

    The synchronisation with the robot is automatically launched at instanciation and is handled in background automatically.
    """

    def __init__(self, host: str, sdk_port: int = 50055, camera_port: int = 50057) -> None:
        """Set up the connection with the robot."""
        self._host = host
        self._sdk_port = sdk_port
        self._camera_port = camera_port
        self._grpc_channel = grpc.insecure_channel(f'{self._host}:{self._sdk_port}')

        self.joints: List[Joint] = []
        self.fans: List[Fan] = []
        self.force_sensors: List[ForceSensor] = []

        self._setup_joints()
        self._setup_arms()
        self._setup_fans()
        self._setup_force_sensors()
        self._setup_cameras()

        self._sync_thread = threading.Thread(target=self._start_sync_in_bg)
        self._sync_thread.daemon = True
        self._sync_thread.start()

    def __repr__(self) -> str:
        """Clean representation of a Reachy."""
        s = '\n\t'.join([str(j) for j in self.joints])
        return f'''<Reachy host="{self._host}" joints=\n\t{
            s
        }\n>'''

    def goto(
        self,
        goal_positions: Dict[Joint, float],
        duration: float,
        starting_positions: Optional[Dict[Joint, float]] = None,
        sampling_freq: float = 100,
        interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
    ):
        """Send joints command to move the robot to a goal_positions within the specified duration.

        This function will block until the movement is over. See goto_async for an asynchronous version.

        The goal positions is expressed in joints coordinates. You can use as many joints target as you want.
        The duration is expressed in seconds.
        You can specify the starting_position, otherwise its current position is used,
        for instance to start from its goal position and avoid bumpy start of move.
        The sampling freq sets the frequency of intermediate goal positions commands.
        You can also select an interpolation method use (linear or minimum jerk) which will influence directly the trajectory.

        """
        def _wrapped_goto(pos):
            asyncio.run(
                self.goto_async(
                    goal_positions=goal_positions,
                    duration=duration,
                    starting_positions=starting_positions,
                    sampling_freq=sampling_freq,
                    interpolation_mode=interpolation_mode,
                )
            )

        with ThreadPoolExecutor() as exec:
            exec.submit(_wrapped_goto, -20)

    async def goto_async(
        self,
        goal_positions: Dict[Joint, float],
        duration: float,
        starting_positions: Optional[Dict[Joint, float]] = None,
        sampling_freq: float = 100,
        interpolation_mode: InterpolationMode = InterpolationMode.LINEAR,
    ):
        """Send joints command to move the robot to a goal_positions within the specified duration.

        This function is asynchronous and will return a Future. This can be used to easily combined multiple gotos.
        See goto for an blocking version.

        The goal positions is expressed in joints coordinates. You can use as many joints target as you want.
        The duration is expressed in seconds.
        You can specify the starting_position, otherwise its current position is used,
        for instance to start from its goal position and avoid bumpy start of move.
        The sampling freq sets the frequency of intermediate goal positions commands.
        You can also select an interpolation method use (linear or minimum jerk) which will influence directly the trajectory.

        """
        if starting_positions is None:
            starting_positions = {j: j.present_position for j in goal_positions.keys()}

        length = round(duration * sampling_freq)
        if length < 1:
            raise ValueError('Goto length too short! (incoherent duration {duration} or sampling_freq {sampling_freq})!')

        joints = starting_positions.keys()
        dt = 1 / sampling_freq

        traj_func = interpolation_mode(
            np.array(list(starting_positions.values())),
            np.array(list(goal_positions.values())),
            duration,
        )

        t0 = time.time()
        while True:
            elapsed_time = time.time() - t0
            if elapsed_time > duration:
                break

            point = traj_func(elapsed_time)
            for j, pos in zip(joints, point):
                j.goal_position = pos

            await asyncio.sleep(dt)

    def _setup_joints(self):
        joint_stub = joint_pb2_grpc.JointServiceStub(self._grpc_channel)

        joint_ids = joint_stub.GetAllJointsId(Empty())
        self._joint_uid_to_name = dict(zip(joint_ids.uids, joint_ids.names))
        self._joint_name_to_uid = dict(zip(joint_ids.names, joint_ids.uids))

        req = joint_pb2.JointsStateRequest(
            ids=[joint_pb2.JointId(uid=uid) for uid in joint_ids.uids],
            requested_fields=[joint_pb2.JointField.ALL],
        )
        joints_state = joint_stub.GetJointsState(req)

        for joint_id, joint_state in zip(joints_state.ids, joints_state.states):
            joint = Joint(joint_state)

            self.joints.append(joint)
            setattr(self, joint.name, joint)

    def _setup_arms(self):
        try:
            left_arm = LeftArm(self.joints, self._grpc_channel)
            setattr(self, 'l_arm', left_arm)
        except ValueError:
            pass

        try:
            right_arm = RightArm(self.joints, self._grpc_channel)
            setattr(self, 'r_arm', right_arm)
        except ValueError:
            pass

    def _setup_fans(self):
        self._fans_stub = fan_pb2_grpc.FanControllerServiceStub(self._grpc_channel)
        resp = self._fans_stub.GetAllFansId(Empty())
        for name, uid in zip(resp.names, resp.uids):
            fan = Fan(name, uid, stub=self._fans_stub)
            setattr(self, name, fan)
            self.fans.append(fan)

    def _setup_force_sensors(self):
        force_stub = sensor_pb2_grpc.SensorServiceStub(self._grpc_channel)
        resp = force_stub.GetAllForceSensorsId(Empty())
        names, uids = resp.names, resp.uids

        resp = force_stub.GetSensorsState(
            sensor_pb2.SensorsStateRequest(ids=[sensor_pb2.SensorId(uid=uid) for uid in uids]),
        )
        states = resp.states
        for name, uid, state in zip(names, uids, states):
            force_sensor = ForceSensor(name, uid, state.force_sensor_state)
            setattr(self, name, force_sensor)
            self.force_sensors.append(force_sensor)

    def _setup_cameras(self):
        try:
            channel = grpc.insecure_channel(f'{self._host}:{self._camera_port}')
            stub = camera_reachy_pb2_grpc.CameraServiceStub(channel)
            self.left_camera = Camera(side='left', stub=stub)
            self.right_camera = Camera(side='right', stub=stub)

        except grpc.RpcError:
            pass

    async def _poll_waiting_commands(self):
        await asyncio.wait(
            [asyncio.create_task(joint._need_sync.wait()) for joint in self.joints],
            return_when=asyncio.FIRST_COMPLETED,
        )
        return joint_pb2.JointsCommand(
            commands=[
                joint._pop_command()
                for joint in self.joints if joint._need_sync.is_set()
            ],
        )

    def _start_sync_in_bg(self):
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self._sync_loop())

    async def _get_stream_update_loop(self, joint_stub, fields, freq: float):
        stream_req = joint_pb2.StreamJointsRequest(
            request=joint_pb2.JointsStateRequest(
                ids=[joint_pb2.JointId(uid=joint.uid) for joint in self.joints],
                requested_fields=fields,
            ),
            publish_frequency=freq,
        )
        async for state_update in joint_stub.StreamJointsState(stream_req):
            for joint_id, state in zip(state_update.ids, state_update.states):
                joint = self._get_joint_from_id(joint_id)
                joint._update_with(state)

    async def _get_stream_sensor_loop(self, async_channel, freq: float):
        stub = sensor_pb2_grpc.SensorServiceStub(async_channel)
        stream_req = sensor_pb2.StreamSensorsStateRequest(
            request=sensor_pb2.SensorsStateRequest(
                ids=[sensor_pb2.SensorId(uid=sensor.uid) for sensor in self.force_sensors],
            ),
            publish_frequency=freq,
        )
        async for state_update in stub.StreamSensorStates(stream_req):
            for sensor_id, state in zip(state_update.ids, state_update.states):
                sensor = self._get_sensor_from_id(sensor_id)
                sensor._update_with(state.force_sensor_state)

    async def _stream_commands_loop(self, joint_stub, freq: float):
        async def command_poll():
            last_pub = 0
            dt = 1.0 / freq

            while True:
                elapsed_time = time.time() - last_pub
                if elapsed_time < dt:
                    await asyncio.sleep(dt - elapsed_time)

                commands = await self._poll_waiting_commands()
                yield commands
                last_pub = time.time()

        await joint_stub.StreamJointsCommands(command_poll())

    async def _sync_loop(self):
        for joint in self.joints:
            joint._setup_sync_loop()

        async_channel = grpc.aio.insecure_channel(f'{self._host}:{self._sdk_port}')
        joint_stub = joint_pb2_grpc.JointServiceStub(async_channel)

        await asyncio.gather(
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.PRESENT_POSITION], freq=100),
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.TEMPERATURE], freq=0.1),
            self._get_stream_sensor_loop(async_channel, freq=10),
            self._stream_commands_loop(joint_stub, freq=100),
        )

    def _get_joint_from_id(self, joint_id: joint_pb2.JointId) -> Joint:
        if joint_id.HasField('uid'):
            return getattr(self, self._joint_uid_to_name[joint_id.uid])
        else:
            return getattr(self, joint_id.name)

    def _get_sensor_from_id(self, sensor_id: sensor_pb2.SensorId) -> ForceSensor:
        if sensor_id.HasField('uid'):
            for sensor in self.force_sensors:
                if sensor.uid == sensor_id.uid:
                    return sensor
            raise IndexError
        else:
            return getattr(self, sensor_id.name)
