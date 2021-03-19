import time
from concurrent.futures import ThreadPoolExecutor

import grpc
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue, UInt32Value

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api.joint_pb2 import JointField


class MockupJoint:
    def __init__(self):
        self.values = {}

    def get(self, field):
        return self.values[field]


class MockupSDKServer(joint_pb2_grpc.JointServiceServicer):
    def __init__(self):
        self.joints = {
            'l_shoulder_pitch': MockupJoint(),
            'l_shoulder_roll': MockupJoint(),
        }
        self.joint_uid_to_name = {uid: name for uid, name in enumerate(self.joints.keys())}
        for uid, name in self.joint_uid_to_name.items():
            self.joints[name].values['name'] = name
            self.joints[name].values['uid'] = UInt32Value(value=uid)

            self.joints[name].values['present_position'] = FloatValue(value=0.0)
            self.joints[name].values['present_speed'] = FloatValue(value=0.0)
            self.joints[name].values['present_load'] = FloatValue(value=0.0)
            self.joints[name].values['temperature'] = FloatValue(value=42.0)

            self.joints[name].values['compliant'] = BoolValue(value=False)
            self.joints[name].values['goal_position'] = FloatValue(value=0.0)
            self.joints[name].values['speed_limit'] = FloatValue(value=100.0)
            self.joints[name].values['torque_limit'] = FloatValue(value=100.0)
            self.joints[name].values['pid'] = joint_pb2.PIDValue(pid=joint_pb2.PIDGains())

    def parse_joint_fields(self, fields):
        """Parse JointField (handles NONE, ALL specific cases)."""
        if JointField.NONE in fields:
            return []

        if JointField.ALL in fields:
            fields = JointField.values()
            fields.remove(JointField.ALL)
            fields.remove(JointField.NONE)

        return fields

    def jointstate_pb_from_request(self, joint, fields):
        fields = self.parse_joint_fields(fields)

        params = {}
        for field in fields:
            name = JointField.Name(field).lower()
            params[name] = joint.get(name)

        return joint_pb2.JointState(**params)

    def GetAllJointsId(self, request, context):
        """Get all the joints name."""
        return joint_pb2.JointsId(
            names=self.joint_uid_to_name.values(), 
            uids=self.joint_uid_to_name.keys(),
        )

    def GetJointsState(self, request, context):
        """Get the requested joints id."""
        params = {}

        params['ids'] = [joint_pb2.JointId(uid=id.uid) for id in request.ids]
        params['states'] = [
            self.jointstate_pb_from_request(
                self.joints[self.joint_uid_to_name[id.uid]],
                request.requested_fields,
            )
            for id in request.ids
        ]
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return joint_pb2.JointsState(**params)

    def StreamJointsState(self, request: joint_pb2.StreamJointsRequest, context):
        """Continuously stream requested joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            joints_state = self.GetJointsState(request.request, context)
            joints_state.timestamp.GetCurrentTime()

            yield joints_state
            last_pub = time.time()

    def StreamJointsCommands(self, request_iterator, context):
        for req in request_iterator:
            print('Salut', req)

        return joint_pb2.JointsCommandAck()

if __name__ == '__main__':
    mockup_sdk_server = MockupSDKServer()

    grpc_server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=20))

    joint_pb2_grpc.add_JointServiceServicer_to_server(mockup_sdk_server, grpc_server)

    grpc_server.add_insecure_port('[::]:50055')
    grpc_server.start()

    grpc_server.wait_for_termination()
