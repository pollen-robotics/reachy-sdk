"""
Example on how to get both camera flux from reachy_sdk_server and display it.

Just press 's' to switch from left to right camera.
"""
import grpc
from reachy_sdk_api import ros_cam_pb2_grpc, ros_cam_pb2
import numpy as np
import cv2 as cv


def switch_side(side):
    """Toggle the camera displayed."""
    if side == 'left':
        return 'right'
    return 'left'


def run():
    """Main."""
    options = [('grpc.max_send_message_length', 200000), ('grpc.max_receive_message_length', 200000)]
    side = 'right'
    host = 'localhost'
    port = 50051
    with grpc.insecure_channel(f'{host}:{port}', options=options) as channel:
        stub = ros_cam_pb2_grpc.CameraServiceStub(channel)
        while True:
            response = stub.GetImage(ros_cam_pb2.Side(side=side))
            img = np.frombuffer(response.data, dtype=np.uint8)
            img = cv.imdecode(img, cv.IMREAD_COLOR)
            cv.imshow('frame',img)
            if cv.waitKey(1) & 0xFF == ord('s'):
                side = switch_side(side)


if __name__ == '__main__':
    run()
