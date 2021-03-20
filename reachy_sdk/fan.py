"""This module define the Fan class and method to get/set their state."""

from reachy_sdk_api import fan_pb2, fan_pb2_grpc


class Fan:
    """The Fan class and get/set state methods.

    The Fan class is used to store the up-to-date state of the fan.
    You can get its current state (check if they were automatically turned on by a temperature limit detection).
    You can also turn them on/off whenever you want.
    """

    def __init__(self, name: str, uid: int, stub: fan_pb2_grpc.FanControllerServiceStub) -> None:
        """Set up the fan."""
        self._name = name
        self._uid = uid

        self._stub = stub

    def __repr__(self) -> str:
        """Clean representation of a fan state."""
        state = 'on' if self.is_on else 'off'
        return f'<Fan name="{self.name}" state="{state}">'

    @property
    def name(self):
        """Get the name of the fan."""
        return self._name

    @property
    def is_on(self):
        """Check if the fan is on."""
        resp = self._stub.GetFansState(fan_pb2.FansStateRequest(
            ids=[fan_pb2.FanId(uid=self._uid)],
        ))
        return resp.states[0].on

    def on(self):
        """Turn the fan on."""
        self._set_state(on=True)

    def off(self):
        """Turn the fan off."""
        self._set_state(on=False)

    def _set_state(self, on: bool):
        resp = self._stub.SendFansCommands(fan_pb2.FansCommand(
            commands=[
                fan_pb2.FanCommand(
                    id=fan_pb2.FanId(uid=self._uid),
                    on=on,
                ),
            ]
        ))
        if not resp.success:
            state = 'on' if self.is_on else 'off'
            raise IOError(f'Could not set fan "{self.name}" {state}!')
