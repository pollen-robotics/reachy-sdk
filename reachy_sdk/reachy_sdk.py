import time
from typing import List

import grpc
from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api import joint_command_pb2_grpc,  joint_state_pb2_grpc
from reachy_sdk_api.joint_state_pb2 import JointStateField, JointRequest, StreamAllJointsRequest
from reachy_sdk_api.joint_command_pb2 import JointCommand, MultipleJointsCommand

from .joint import Joint


class ReachySDK:
    def __init__(self, host: str = 'localhost', port: int = 50051, sync_freq: float = 100) -> None:
        self._channel = grpc.insecure_channel(f'{host}:{port}')
        self._joint_state_stub = joint_state_pb2_grpc.JointStateServiceStub(self._channel)
        self._joint_command_stub = joint_command_pb2_grpc.JointCommandServiceStub(self._channel)

        self.joints: List[Joint] = []
        self._get_initial_joint_state()

        self._sync_freq = sync_freq
        self._run_sync_loop_in_bg()

    def _get_initial_joint_state(self) -> None:
        self.joints.clear()

        motor_names = self._joint_state_stub.GetAllJointNames(Empty())

        for id, name in enumerate(motor_names.names):
            joint_state = self._joint_state_stub.GetJointState(JointRequest(
                name=name, requested_fields=[JointStateField.ALL],
            ))
            j = Joint(
                name=joint_state.name,
                id=id,
                present_position=joint_state.present_position.value,
                present_speed=joint_state.present_speed.value,
                present_load=joint_state.present_load.value,
                temperature=joint_state.temperature.value,
                goal_position=joint_state.goal_position.value,
                speed_limit=joint_state.speed_limit.value,
                torque_limit=joint_state.torque_limit.value,
                compliant=joint_state.compliant.value,
                # pid=joint_state.pid.value,
            )
            setattr(self, name, j)
            self.joints.append(j)

    def _run_sync_loop_in_bg(self) -> None:
        from threading import Thread

        t = []
        t.append(Thread(target=self._get_position_updates))
        t.append(Thread(target=self._get_temperature_updates))
        # t.append(Thread(target=self._send_commands))
        t.append(Thread(target=self._stream_commands))

        for tt in t:
            tt.daemon = True
            tt.start()

    def _get_position_updates(self) -> None:
        req = StreamAllJointsRequest(
            requested_fields=[JointStateField.PRESENT_POSITION],
            publish_frequency=self._sync_freq,
        )

        for update in self._joint_state_stub.StreamAllJointsState(req):
            for joint, joint_state in zip(self.joints, update.joints):
                joint._fields['present_position'].value = joint_state.present_position.value

    def _get_temperature_updates(self) -> None:
        req = StreamAllJointsRequest(
            requested_fields=[JointStateField.TEMPERATURE],
            publish_frequency=0.1,
        )
        for update in self._joint_state_stub.StreamAllJointsState(req):
            for joint, joint_state in zip(self.joints, update.joints):
                joint._fields['temperature'].value = joint_state.temperature.value

    def _stream_commands(self) -> None:
        def cmd_gen():
            while True:
                yield MultipleJointsCommand(commands=self._waiting_commands())
                time.sleep(1.0 / self._sync_freq)

        self._joint_command_stub.StreamJointsCommand(cmd_gen())

    def _send_commands(self) -> None:
        while True:
            for cmd in self._waiting_commands():
                self._joint_command_stub.SendCommand(cmd)
            time.sleep(1.0 / self._sync_freq)

    def _waiting_commands(self) -> List[JointCommand]:
        return [
            joint._pop_sync_command()
            for joint in self.joints if joint._need_sync()
        ]
