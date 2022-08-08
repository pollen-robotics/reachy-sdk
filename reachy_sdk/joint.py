"""This module define the Joint class (any dynamixel motor or one of orbita's disk) and its registers."""

import asyncio
from typing import Any, Dict, List

import numpy as np

from google.protobuf.wrappers_pb2 import BoolValue, FloatValue, UInt32Value

from reachy_sdk_api.joint_pb2 import JointId, JointState, JointCommand, PIDValue

from .register import MetaRegister, Register


def _to_position(internal_pos: float) -> float:
    return round(np.rad2deg(internal_pos), 2)


def _to_internal_position(pos: float) -> float:
    return np.deg2rad(pos)


class Joint(metaclass=MetaRegister):
    """The Joint class represents (any dynamixel motor or one of orbita's disk) and its registers.

    The Joint class is used to store the up-to-date state of the joint, especially the RO registers:
        - its name and uid (resp. str and int) which will never change
        - its current position (in degree)
        - the current speed and load are presently not updated as the value given by the motor is unreliable
        - its current temperature (in degree celsius)

    Similarly, when setting a new value to one of its RW register.
    The Joint is responsible to inform the robot that some commands need to be sent.
    The RW registers are:
        - set the compliance on/off (boolean)
        - goal position (in degree)
        - speed limit (in degree per second)
        - the torque limit (in %)
        - its pid (refer to the different types of motor to have a better understanding of the gain expected range)
    """

    name = Register(readonly=True, type=str)
    uid = Register(readonly=True, type=UInt32Value)

    present_position = Register(readonly=True, type=FloatValue, conversion=(_to_internal_position, _to_position))
    present_speed = Register(readonly=True, type=FloatValue, conversion=(_to_internal_position, _to_position))
    present_load = Register(readonly=True, type=FloatValue)
    temperature = Register(readonly=True, type=FloatValue)

    compliant = Register(readonly=False, type=BoolValue)
    goal_position = Register(readonly=False, type=FloatValue, conversion=(_to_internal_position, _to_position))
    speed_limit = Register(readonly=False, type=FloatValue, conversion=(_to_internal_position, _to_position))
    torque_limit = Register(readonly=False, type=FloatValue)

    pid = Register(readonly=False, type=PIDValue)

    def __init__(self, initial_state: JointState) -> None:
        """Set up the joint with its initial state retrieved from the robot."""
        self._state: Dict[str, Any] = {}

        self._update_with(initial_state)

    def __repr__(self):
        """Clean representation of a joint basic state."""
        mode = "compliant" if self.compliant else "stiff"
        return f'<Joint name="{self.name}" pos="{self.present_position:.2f}" mode="{mode}">'

    def registers(self):
        """Return a dict[name, value] of all the Joint registers."""
        return {
            field: getattr(self, field)
            for field in self._state.keys()
        }

    def __getitem__(self, field: str) -> Any:
        """Access a specific register of the Joint using the joint[register_name] notation."""
        return self._state[field]

    def __setitem__(self, field: str, value: Any):
        """Set a new value for a register of the Joint using the joint[register_name] = value notation."""
        self._state[field] = value

        if field == 'compliant' and not value.value:
            self._state['goal_position'] = self._state['present_position']

        async def set_in_loop():
            self._register_needing_sync.append(field)
            self._need_sync.set()

        fut = asyncio.run_coroutine_threadsafe(set_in_loop(), self._loop)
        fut.result()

    def _setup_sync_loop(self):
        """Set up the async synchronisation loop.

        The setup is done separately, as the async Event should be created in the same EventLoop than it will be used.

        The _need_sync Event is used to inform the robot that some data need to be pushed to the real robot.
        The _register_needing_sync stores a list of the register that need to be synced.
        """
        self._need_sync = asyncio.Event()
        self._loop = asyncio.get_running_loop()
        self._register_needing_sync: List[str] = []

    def _update_with(self, new_state: JointState):
        """Update the joint with a newly received (partial) state received from the gRPC server."""
        for field, value in new_state.ListFields():
            self._state[field.name] = value

    def _pop_command(self) -> JointCommand:
        """Create a gRPC command from the registers that need to be synced."""
        values = {
            'id': JointId(uid=self.uid)
        }
        values.update({
            reg: self._state[reg]
            for reg in self._register_needing_sync
        })
        command = JointCommand(**values)

        self._register_needing_sync.clear()
        self._need_sync.clear()

        return command
