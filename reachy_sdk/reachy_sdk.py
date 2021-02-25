"""Reachy SDK."""
import time
from typing import List
import cv2 as cv
import numpy as np

import grpc
from google.protobuf.empty_pb2 import Empty

from reachy_sdk_api import joint_command_pb2_grpc,  joint_state_pb2_grpc
from reachy_sdk_api import camera_reachy_pb2_grpc, load_sensor_pb2_grpc
from reachy_sdk_api import arm_kinematics_pb2_grpc, zoom_command_pb2_grpc
from reachy_sdk_api import cartesian_command_pb2_grpc, orbita_kinematics_pb2_grpc

from reachy_sdk_api.joint_state_pb2 import JointStateField, JointRequest, StreamAllJointsRequest
from reachy_sdk_api.joint_command_pb2 import JointCommand, MultipleJointsCommand
from reachy_sdk_api.camera_reachy_pb2 import Side as CamSide
from reachy_sdk_api.load_sensor_pb2 import Side as LoadSide
from reachy_sdk_api.arm_kinematics_pb2 import ArmEndEffector, ArmJointsPosition, ArmSide
from reachy_sdk_api.kinematics_pb2 import JointsPosition, Matrix4x4
from reachy_sdk_api.zoom_command_pb2 import ZoomCommand, ZoomSpeed
from reachy_sdk_api.orbita_kinematics_pb2 import Point
from reachy_sdk_api.cartesian_command_pb2 import FullBodyCartesianCommand

from .joint import Joint


side_to_proto = {'left': ArmSide.LEFT, 'right': ArmSide.RIGHT}


class ReachySDK:
    def __init__(self, host: str = 'localhost', port: int = 50055, sync_freq: float = 100) -> None:
        options = [('grpc.max_send_message_length', 200000), ('grpc.max_receive_message_length', 200000)]
        self._channel = grpc.insecure_channel(f'{host}:{port}', options=options)
        self._joint_state_stub = joint_state_pb2_grpc.JointStateServiceStub(self._channel)
        self._joint_command_stub = joint_command_pb2_grpc.JointCommandServiceStub(self._channel)
        self._load_sensor_stub = load_sensor_pb2_grpc.LoadServiceStub(self._channel)
        self._camera_stub = camera_reachy_pb2_grpc.CameraServiceStub(self._channel)
        self._arm_kinematics_stub = arm_kinematics_pb2_grpc.ArmKinematicStub(self._channel)
        self._zoom_controller_stub = zoom_command_pb2_grpc.ZoomControllerServiceStub(self._channel)
        self._orbita_stub = orbita_kinematics_pb2_grpc.OrbitaKinematicStub(self._channel)
        self._cartesian_stub = cartesian_command_pb2_grpc.CartesianCommandServiceStub(self._channel)

        self.joints: List[Joint] = []
        self._get_initial_joint_state()

        self._sync_freq = sync_freq
        self._run_sync_loop_in_bg()

        self.left_image = None
        self.right_image = None

        self.left_load_sensor = 0
        self.right_load_sensor = 0

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
        t.append(Thread(target=self._get_load_sensor_updates))
        # t.append(Thread(target=self._send_commands))
        t.append(Thread(target=self._stream_commands))
        t.append(Thread(target=self._get_image))

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

    def _get_load_sensor_updates(self) -> None:
        while True:
            self.left_load_sensor = self._load_sensor_stub.GetLoad(LoadSide(side='left')).load
            self.right_load_sensor = self._load_sensor_stub.GetLoad(LoadSide(side='right')).load

    def _stream_commands(self) -> None:
        def cmd_gen():
            while True:
                commands = self._waiting_commands()
                if commands:
                    yield MultipleJointsCommand(commands=commands)
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

    def _get_image(self):
        while True:
            self.left_image = self._response_to_img(side='left')
            self.right_image = self._response_to_img(side='right')

    def _response_to_img(self, side):
        response = self._camera_stub.GetImage(CamSide(side=side))
        img = np.frombuffer(response.data, dtype=np.uint8)
        img = cv.imdecode(img, cv.IMREAD_COLOR)
        return img

    def forward_kinematics(self, arm_side, joints_positions):
        '''Get from the sdk_server the forward kinematics for a given arm.

        Args:
            arm_side (str): required arm
            joints_positions ([float]): list of the present_position of each joints.
                len(joints_positions) should be equal to 7.

        Returns:
            :py:class:'~numpy.ndarray': 4x4 homogeneous matrix pose of the end effector
        '''
        joints = JointsPosition(positions=joints_positions)

        req = ArmJointsPosition(
            side=side_to_proto[arm_side],
            positions=joints
            )
        response = self._arm_kinematics_stub.ComputeArmFK(req)
        return np.array(response.target.data).reshape((4,4))

    def inverse_kinematics(self, arm_side, target, q0):
        '''Get from the sdk_server the inverse kinematics for a given arm.

        Args:
            arm_side (str): required arm
            target (:py:class:`~numpy.ndarray`): 4*4 homogeneous matrix pose for the end effector
            q0 ([float]): list containing the present_position of each motor joint. 
                q0 is used to optimize the inverse kinematics computation time.

        Returns:
            [float]: list containing the goal_position of each motor joint to reach the target
        '''
        joints = JointsPosition(positions=q0)
        target_proto = Matrix4x4(data=np.ndarray.flatten(target))

        req = ArmEndEffector(
            side=side_to_proto[arm_side],
            target=target_proto,
            q0=joints
            )
        response = self._arm_kinematics_stub.ComputeArmIK(req)
        return response.positions.positions

    def zoom_command(self, side, command):
        '''Send command to Reachy's zoom.
        
        Args:
            side (str): which camera to send the command to. ('right' or 'left')
            command (str): type of command to send. ('homing', 'in', 'inter' or 'out')
                'homing': the required zoom camera will perform homing. 
                    Do it once before changing the zoom level.
                'in': change zoom to see far objects
                'out': change zoom to see close objects
                'inter': zoom level between 'in' and 'out'
        '''
        req = ZoomCommand(
            side=side,
            command=command
        )
        self._zoom_controller_stub.SendZoomCommand(req)

    def set_zoom_speed(self, speed):
        '''Change the speed of zoom motors.
        
        Args:
            speed (int): motors speed, must be between 4000 and 40000.
        '''
        self._zoom_controller_stub.SetZoomSpeed(ZoomSpeed(speed=speed))
        
    def look_at(self, x: float, y: float, z: float):
        '''Perform look_at on Orbita.

        Args:
            x, y, z (float): coordinates of the point to look_at in the robot frame
        '''
        quat = self._orbita_stub.GetQuaternionTransform(
            Point(
                x=x,
                y=y,
                z=z,
            )
        )
        self._cartesian_stub.SendCartesianCommand(
            FullBodyCartesianCommand(
                orbita_target=quat
            )
        )
