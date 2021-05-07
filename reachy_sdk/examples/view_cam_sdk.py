"""Test Reachy's cameras usig Reachy's SDK.

Make sure that reachy_sdk_server is running before using this.
"""
import cv2 as cv

from reachy_sdk import ReachySDK


class SDKCameraViewer:
    """Camera Viewer using Reachy's SDK."""

    def __init__(self, side: str, ip_address: str = 'localhost') -> None:
        """Connect to the requested camera of the robot.

        Args:
            - side: 'right' or 'left',
            - ip_address: ip_address of the robot. Default is 'localhost'.
        """
        try:
            self.reachy = ReachySDK(ip_address)
        except Exception:
            print(
                'Could not connect to Reachy. Make sure that:',
                '   - You are connected on the same wifi network,',
                '   - You are using the correct IP address,',
                '   - launch_all.bash in reachy_sdk_server has actually been launched.',
                sep='\n'
            )
            quit()

        self.camera = getattr(self.reachy, f'{side}_camera')
        self.last_image = self.camera.last_frame

    def get_last_image(self):
        """Recover the last frame from the robot's camera."""
        self.last_image = self.camera.last_frame


def main():
    """Instanciate the CameraViewer object for the requested side."""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('side', help="Reachy's camera requested, either 'left' or 'right'.")
    parser.add_argument('--ip_address', help="Reachy's ip address, default is 'localhost'.", default='localhost')
    args = parser.parse_args()

    requested_side = args.side

    if requested_side not in ['left', 'right']:
        raise ValueError("side argument must either be 'left' or 'right'")

    image_getter = SDKCameraViewer(side=args.side, ip_address=args.ip_address)
    while True:
        image_getter.get_last_image()
        cv.imshow(args.side + ' camera', image_getter.last_image)
        if cv.waitKey(33) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
