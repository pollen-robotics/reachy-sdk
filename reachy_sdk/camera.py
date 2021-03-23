"""Camera Reachy module.

This module lets you access always up-to-date image from the camera.
The synchronisation is done in background and only started when you need it.
"""

from enum import Enum
from typing import Optional
import cv2 as cv
import numpy as np

from threading import Event, Thread

from reachy_sdk_api import camera_reachy_pb2, camera_reachy_pb2_grpc


class ZoomLevel(Enum):
    """The zoom level options."""

    IN = camera_reachy_pb2.ZoomLevelCommand.IN
    INTER = camera_reachy_pb2.ZoomLevelCommand.INTER
    OUT = camera_reachy_pb2.ZoomLevelCommand.OUT


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

        self._camera = camera_reachy_pb2.Camera.LEFT if side == 'left' else camera_reachy_pb2.Camera.RIGHT
        self._stub = stub

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

    def zoom_homing(self):
        """Run a homing to reset zoom position."""
        ...


    @property
    def zoom_speed(self) -> int:
        pass

    @zoom_speed.setter
    def zoom_speed(self, speed: int):
        pass

    @property
    def zoom_level(self) -> ZoomLevel:
        pass

    @zoom_level.setter
    def zoom_level(self, lvl: ZoomLevel):
        pass

    def _start_sync_in_bg(self):
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
