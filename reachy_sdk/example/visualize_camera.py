"""
Example on how to get the camera image from reachy_sdk_server and display it.
"""
import grpc
from reachy_sdk_api import camera_pb2_grpc, camera_pb2
import numpy as np
import cv2 as cv


def run():
    host = 'localhost'
    port = 50051
    options = [
        ('grpc.max_send_message_length', 512 * 1024 * 1024),
        ('grpc.max_receive_message_length', 512 * 1024 * 1024)]

    with grpc.insecure_channel(f'{host}:{port}', options=options) as channel:
        stub = camera_pb2_grpc.CameraServiceStub(channel)
        while True:
            response = stub.GetImage(camera_pb2.Side(side=0))
            img = np.frombuffer(response.data, dtype=np.uint8)
            img = cv.imdecode(img, cv.IMREAD_COLOR)
            cv.imshow('camera_frame', img)
            cv.waitKey(1)


if __name__ == '__main__':
    run()
