import asyncio
import threading
import time
from typing import List

import grpc
from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api import joint_pb2, joint_pb2_grpc

from .joint import Joint


class ReachySDK:
    def __init__(self, host: str, sdk_port: int = 50055) -> None:
        self._host = host
        self._sdk_port = sdk_port

        self.joints: List[Joint] = []
        self._setup_all_joints()

        self._sync_thread = threading.Thread(target=self._start_sync_in_bg)
        self._sync_thread.daemon = True
        self._sync_thread.start()

    def _setup_all_joints(self):
        channel = grpc.insecure_channel(f'{self._host}:{self._sdk_port}')
        joint_stub = joint_pb2_grpc.JointServiceStub(channel)

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

        channel = grpc.aio.insecure_channel(f'{self._host}:{self._sdk_port}')
        joint_stub = joint_pb2_grpc.JointServiceStub(channel)

        await asyncio.gather(
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.PRESENT_POSITION], freq=100),
            self._get_stream_update_loop(joint_stub, fields=[joint_pb2.JointField.TEMPERATURE], freq=0.1),
            self._stream_commands_loop(joint_stub, freq=1),
        )

    def _get_joint_from_id(self, joint_id: joint_pb2.JointId) -> Joint:
        if joint_id.HasField('uid'):
            return getattr(self, self._joint_uid_to_name[joint_id.uid])
        else:
            return getattr(self, joint_id.name)
