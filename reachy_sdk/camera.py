"""Camera Reachy module.

This module lets you access always up-to-date image from the camera.
The synchronisation is done in background and only started when you need it.
"""

from enum import Enum
from typing import Optional
import cv2 as cv
from google.protobuf.empty_pb2 import Empty
from google.protobuf.wrappers_pb2 import UInt32Value
import numpy as np

from threading import Event, Thread

from reachy_sdk_api import camera_reachy_pb2, camera_reachy_pb2_grpc


class ZoomLevel(Enum):
    """The zoom level options."""

    ZERO = camera_reachy_pb2.ZoomLevelPossibilities.ZERO
    IN = camera_reachy_pb2.ZoomLevelPossibilities.IN
    INTER = camera_reachy_pb2.ZoomLevelPossibilities.INTER
    OUT = camera_reachy_pb2.ZoomLevelPossibilities.OUT


class Camera:
    """The camera class."""

    def __init__(self,
                 side: str, stub: camera_reachy_pb2_grpc.CameraServiceStub,
                 ) -> None:
        """Set up the camera.

        The sync loop is only started the first time you try to access a frame.
        This lazy set up allows for not streaming the images when they are not used.
        """
        self._initialized = False
        self._got_img = Event()
        self._last_frame: Optional[np.ndarray] = None

        self._side = side
        self._camera = camera_reachy_pb2.Camera(
            id=camera_reachy_pb2.CameraId.LEFT if side == 'left' else camera_reachy_pb2.CameraId.RIGHT,
        )

        self._stub = stub

    def __repr__(self) -> str:
        """Clean representation of a camera."""
        return f'<Camera side="{self._side}" resolution={self.last_frame.shape}>'

    @property
    def last_frame(self) -> np.ndarray:
        """Return the last retrieved frame."""
        if not self._initialized:
            self._start_sync_in_bg()

        assert self._last_frame is not None

        return self._last_frame

    def wait_for_new_frame(self) -> np.ndarray:
        """Block until the reception of a new frame."""
        if not self._initialized:
            self._start_sync_in_bg()

        self._got_img.wait()
        self._got_img.clear()
        return self.last_frame

    def zoom_homing(self) -> bool:
        """Run a homing to reset zoom position."""
        resp = self._stub.SendZoomCommand(
            camera_reachy_pb2.ZoomCommand(
                camera=self._camera,
                homing_command=camera_reachy_pb2.ZoomHoming(),
            ),
        )
        return resp.success

    @property
    def zoom_level(self) -> ZoomLevel:
        """Get the current zoom level."""
        resp = self._stub.GetZoomLevel(self._camera)
        return ZoomLevel(resp.level)

    @zoom_level.setter
    def zoom_level(self, level: ZoomLevel):
        if level == ZoomLevel.ZERO:
            raise ValueError('Can not set zoom level to zero, use homing instead!')

        resp = self._stub.SendZoomCommand(
            camera_reachy_pb2.ZoomCommand(
                camera=self._camera,
                level_command=camera_reachy_pb2.ZoomLevel(
                    level=level.value,
                ),
            ),
        )
        if not resp.success:
            raise ValueError('Could not set new zoom level!')

    @property
    def zoom_speed(self) -> int:
        """Get the current zoom speed."""
        resp = self._stub.GetZoomSpeed(self._camera)
        return resp.speed

    @zoom_speed.setter
    def zoom_speed(self, speed: int):
        if not (4000 <= speed <= 40000):
            raise ValueError('Speed should be within range (4000, 40000)!')

        resp = self._stub.SendZoomCommand(
            camera_reachy_pb2.ZoomCommand(
                camera=self._camera,
                speed_command=camera_reachy_pb2.ZoomSpeed(
                    speed=speed,
                ),
            ),
        )
        if not resp.success:
            raise ValueError('Could not set new zoom level!')

    def start_autofocus(self):
        """Start autofocus."""
        resp = self._stub.StartAutofocus(self._camera)
        if not resp.success:
            raise EnvironmentError(f'Could not start autofocus for {self._side} camera!')

    def stop_autofocus(self):
        """Stop autofocus."""
        resp = self._stub.StopAutofocus(self._camera)
        if not resp.success:
            raise EnvironmentError(f'Could not start autofocus for {self._side} camera!')

    @property
    def focus(self):
        """Get focus value."""
        resp = self._stub.GetZoomFocus(Empty())
        return getattr(resp, self._side + '_focus').value

    @focus.setter
    def focus(self, focus: int):
        """Set focus value."""
        if self._side == 'left':
            request = camera_reachy_pb2.ZoomFocusMessage(
                left_focus=UInt32Value(value=focus)
                )
        else:
            request = camera_reachy_pb2.ZoomFocusMessage(
                right_focus=UInt32Value(value=focus)
                )
        resp = self._stub.SetZoomFocus(request=request)
        if not resp.success:
            raise ValueError(f'Could not set {self._side} focus!')

    @property
    def zoom(self):
        """Get zoom value."""
        resp = self._stub.GetZoomFocus(Empty())
        return getattr(resp, self._side + '_zoom').value

    @zoom.setter
    def zoom(self, zoom: int):
        """Set zoom value."""
        if self._side == 'left':
            request = camera_reachy_pb2.ZoomFocusMessage(
                left_zoom=UInt32Value(value=zoom)
                )
        else:
            request = camera_reachy_pb2.ZoomFocusMessage(
                right_zoom=UInt32Value(value=zoom)
                )
        resp = self._stub.SetZoomFocus(request=request)
        if not resp.success:
            raise ValueError(f'Could not set {self._side} zoom!')

    def _start_sync_in_bg(self):
        self._initialized = True

        def poll_img():
            stream_request = camera_reachy_pb2.StreamImageRequest(
                request=camera_reachy_pb2.ImageRequest(
                    camera=self._camera,
                ),
            )

            for resp in self._stub.StreamImage(stream_request):
                buff = np.frombuffer(resp.data, dtype=np.uint8)
                self._last_frame = cv.imdecode(buff, cv.IMREAD_COLOR)
                self._got_img.set()

        self._t = Thread(target=poll_img)
        self._t.daemon = True
        self._t.start()

        self._got_img.wait()
