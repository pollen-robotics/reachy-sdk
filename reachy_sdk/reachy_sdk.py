"""ReachySDK package.

This package provides remote access (via socket) to a Reachy robot.
It automatically handles the synchronization with the robot.
In particular, you can easily get an always up-to-date robot state (joint positions, sensors value).
You can also send joint commands, compute forward or inverse kinematics.

"""

import asyncio
import atexit
import threading
import time
from typing import List
from enum import Enum

import grpc
from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api import camera_reachy_pb2_grpc
from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import fan_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import restart_signal_pb2, restart_signal_pb2_grpc

from .arm import LeftArm, RightArm
from .camera import Camera
from .device_holder import DeviceHolder
from .fan import Fan
from .force_sensor import ForceSensor
from .head import Head
from .joint import Joint


class ReachyParts(Enum):
    """Reachy parts options."""

    REACHY = 1
    L_ARM = 2
    R_ARM = 3
    HEAD = 4


class ReachySDK:
    """The ReachySDK class handles the connection with your robot.

    It holds:

    - all joints (can be accessed directly via their name or via the joints list).
    - all force sensors (can be accessed directly via their name or via the force_sensors list).
    - all fans (can be accessed directly via their name or via the fans list).

    The synchronisation with the robot is automatically launched at instanciation and is handled in background automatically.
    """

    def __init__(self, host: str, sdk_port: int = 50055, camera_port: int = 50057, restart_port: int = 50059) -> None:
        """Set up the connection with the robot."""
        self._host = host
        self._sdk_port = sdk_port
        self._camera_port = camera_port
        self._restart_port = restart_port
        self._grpc_channel = grpc.insecure_channel(f'{self._host}:{self._sdk_port}')

        self._joints: List[Joint] = []
        self._fans: List[Fan] = []
        self._force_sensors: List[ForceSensor] = []

        self._ready = threading.Event()
        self._pushed_command = threading.Event()

        self._restart_signal_stub = restart_signal_pb2_grpc.RestartServiceStub(
            grpc.insecure_channel(f'{self._host}:{self._restart_port}')
        )

        self._setup_joints()
        self._setup_arms()
        self._setup_head()
        self._setup_fans()
        self._setup_force_sensors()
        self._setup_cameras()

        self.fans = DeviceHolder(self._fans)
        self.force_sensors = DeviceHolder(self._force_sensors)
        self.joints = DeviceHolder(self._joints)

        self._sync_thread = threading.Thread(target=self._start_sync_in_bg)
        self._sync_thread.daemon = True
        self._sync_thread.start()

        _open_connection.append(self)

        self._ready.wait()

    def __repr__(self) -> str:
        """Clean representation of a Reachy."""
        s = '\n\t'.join([str(j) for j in self._joints])
        return f'''<Reachy host="{self._host}" joints=\n\t{
            s
        }\n>'''

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
            self._joints.append(joint)

    def _setup_arms(self):
        try:
            left_arm = LeftArm(self._joints, self._grpc_channel)
            setattr(self, 'l_arm', left_arm)
        except ValueError:
            pass

        try:
            right_arm = RightArm(self._joints, self._grpc_channel)
            setattr(self, 'r_arm', right_arm)
        except ValueError:
            pass

    def _setup_head(self):
        try:
            head = Head(self._joints, self._grpc_channel)
            setattr(self, 'head', head)
        except ValueError:
            pass

    def _setup_fans(self):
        self._fans_stub = fan_pb2_grpc.FanControllerServiceStub(self._grpc_channel)
        resp = self._fans_stub.GetAllFansId(Empty())
        for name, uid in zip(resp.names, resp.uids):
            fan = Fan(name, uid, stub=self._fans_stub)
            self._fans.append(fan)

    def _setup_force_sensors(self):
        force_stub = sensor_pb2_grpc.SensorServiceStub(self._grpc_channel)
        resp = force_stub.GetAllForceSensorsId(Empty())
        if resp.names == []:
            names, uids, states = [], [], []
        else:
            names, uids = resp.names, resp.uids

            resp = force_stub.GetSensorsState(
                sensor_pb2.SensorsStateRequest(ids=[sensor_pb2.SensorId(uid=uid) for uid in uids]),
            )
            states = resp.states
        for name, uid, state in zip(names, uids, states):
            force_sensor = ForceSensor(name, uid, state.force_sensor_state)
            self._force_sensors.append(force_sensor)

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
            [asyncio.create_task(joint._need_sync.wait()) for joint in self._joints],
            return_when=asyncio.FIRST_COMPLETED,
        )
        return joint_pb2.JointsCommand(
            commands=[
                joint._pop_command()
                for joint in self._joints if joint._need_sync.is_set()
            ],
        )

    def _start_sync_in_bg(self):
        loop = asyncio.new_event_loop()
        loop.run_until_complete(self._sync_loop())

    async def _get_stream_update_loop(self, joint_stub, fields, freq: float):
        stream_req = joint_pb2.StreamJointsRequest(
            request=joint_pb2.JointsStateRequest(
                ids=[joint_pb2.JointId(uid=joint.uid) for joint in self._joints],
                requested_fields=fields,
            ),
            publish_frequency=freq,
        )
        async for state_update in joint_stub.StreamJointsState(stream_req):
            for joint_id, state in zip(state_update.ids, state_update.states):
                joint = self.joints._get_device_from_id(joint_id)
                joint._update_with(state)

    async def _get_stream_sensor_loop(self, async_channel, freq: float):
        stub = sensor_pb2_grpc.SensorServiceStub(async_channel)
        stream_req = sensor_pb2.StreamSensorsStateRequest(
            request=sensor_pb2.SensorsStateRequest(
                ids=[sensor_pb2.SensorId(uid=sensor.uid) for sensor in self._force_sensors],
            ),
            publish_frequency=freq,
        )
        async for state_update in stub.StreamSensorStates(stream_req):
            for sensor_id, state in zip(state_update.ids, state_update.states):
                sensor = self.force_sensors._get_device_from_id(sensor_id)
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
                self._pushed_command.set()
                self._pushed_command.clear()
                last_pub = time.time()

        await joint_stub.StreamJointsCommands(command_poll())

    async def _sync_loop(self):
        for joint in self._joints:
            joint._setup_sync_loop()

        self._ready.set()

        async_channel = grpc.aio.insecure_channel(f'{self._host}:{self._sdk_port}')
        joint_stub = joint_pb2_grpc.JointServiceStub(async_channel)

        await asyncio.gather(
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.PRESENT_POSITION], freq=100),
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.TEMPERATURE], freq=0.1),
            self._get_stream_sensor_loop(async_channel, freq=10),
            self._stream_commands_loop(joint_stub, freq=100),
        )

    def _change_compliancy(self, part, compliant: bool):
        if part not in [p.name.lower() for p in ReachyParts]:
            raise ValueError("Part to turn on/off should be either 'reachy', 'l_arm', 'r_arm' or 'head'.")

        if part == 'reachy':
            req_part = self
        else:
            req_part = getattr(self, part)

        for joint in req_part.joints.values():
            joint.compliant = compliant

    def turn_on(self, part):
        """Turn the joints of the given Reachy's part stiff.

        The requested part can be 'l_arm', 'r_arm', 'head' or 'reachy'.
        Having part = 'reachy' corresponds to turning all avaible joints stiff.
        """
        self._change_compliancy(part, compliant=False)

    def turn_off(self, part):
        """Turn the joints of the given Reachy's part compliant.

        The requested part can be 'l_arm', 'r_arm', 'head' or 'reachy'.
        Having part = 'reachy' corresponds to turning all avaible joints compliant.
        """
        self._change_compliancy(part, compliant=True)

    def turn_off_smoothly(self, part):
        """Turn smoothly the joints of the given Reachy's part compliant.

        Used to prevent parts from falling too hardwhen turned compliant.

        The requested part can be 'l_arm', 'r_arm', 'head' or 'reachy'.
        Having part = 'reachy' corresponds to turning all avaible joints compliant.
        """
        if part not in [p.name.lower() for p in ReachyParts]:
            raise ValueError("Part to turn on/off should be either 'reachy', 'l_arm', 'r_arm' or 'head'.")

        if part == 'reachy':
            req_part = self
        else:
            req_part = getattr(self, part)

        for joint in req_part.joints.values():
            joint.torque_limit = 0.0

        time.sleep(2.0)

        self._change_compliancy(part, compliant=True)

        for joint in req_part.joints.values():
            joint.torque_limit = 100.0

    def _restart(self):
        self._restart_signal_stub.SendRestartSignal(
            restart_signal_pb2.RestartCmd(
                cmd=restart_signal_pb2.SignalType.RESTART
            )
        )

    def _stop(self):
        self._restart_signal_stub.SendRestartSignal(
            restart_signal_pb2.RestartCmd(
                cmd=restart_signal_pb2.SignalType.STOP
            )
        )


_open_connection: List[ReachySDK] = []


def flush_communication():
    """Flush communication before leaving.

    We make sure all buffered commands have been sent before actually leaving.
    """
    for reachy in _open_connection:
        reachy._pushed_command.wait(timeout=0.5)


atexit.register(flush_communication)
