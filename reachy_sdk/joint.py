import numpy as np

from threading import Event
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

from reachy_sdk_api.joint_command_pb2 import JointCommand


class Joint:
    def __init__(self, name: str, id: int,
                 present_position: float, present_speed: float, present_load: float, temperature: float,
                 goal_position: float, speed_limit: float, torque_limit: float,
                 compliant: bool,
                 # pid: (float, float, float),
                 ) -> None:
        self._name = name
        self._id = id

        self._fields = {
            # TODO: Use metaclass instead (look at Django?)
            'present_position': Field('present_position', present_position),
            'present_speed': Field('present_speed', present_speed),
            'present_load': Field('present_load', present_load),
            'temperature': Field('temperature', temperature),

            'goal_position': Field('goal_position', goal_position, FloatValue),
            'speed_limit': Field('speed_limit', speed_limit, FloatValue),
            'torque_limit': Field('torque_limit', torque_limit, FloatValue),
            'compliant': Field('compliant', compliant, BoolValue),
        }

    @property
    def name(self):
        return self._name

    @property
    def present_position(self):
        return np.rad2deg(self._fields['present_position'].value)

    @property
    def present_speed(self):
        return np.rad2deg(self._fields['present_speed'].value)

    @property
    def present_load(self):
        return self._fields['present_load'].value

    @property
    def temperature(self):
        return self._fields['temperature'].value

    @property
    def goal_position(self):
        return np.rad2deg(self._fields['goal_position'].value)

    @goal_position.setter
    def goal_position(self, value):
        self._fields['goal_position'].request_value_update(np.deg2rad(value))

    @property
    def speed_limit(self):
        return np.rad2deg(self._fields['speed_limit'].value)

    @speed_limit.setter
    def speed_limit(self, value):
        self._fields['speed_limit'].request_value_update(np.deg2rad(value))

    @property
    def torque_limit(self):
        return self._fields['torque_limit'].value

    @torque_limit.setter
    def torque_limit(self, value):
        self._fields['torque_limit'].request_value_update(value)

    @property
    def compliant(self):
        return self._fields['compliant'].value

    @compliant.setter
    def compliant(self, value):
        if not value:
            self.goal_position = self.present_position
        self._fields['compliant'].request_value_update(value)

    def _need_sync(self) -> bool:
        for field in self._fields.values():
            if field.sync_evt.is_set():
                return True

        return False

    def _pop_sync_command(self) -> JointCommand:
        args = {'id': self._id}

        for name, field in self._fields.items():
            if field.sync_evt.is_set():
                args[name] = field.protobuf_value_type(value=field.value)
                field.sync_evt.clear()

        return JointCommand(**args)


class Field:
    def __init__(self, name, value, protobuf_value_type=None) -> None:
        self.name = name
        self.value = value
        self.protobuf_value_type = protobuf_value_type

        self.sync_evt = Event()

    def __repr__(self) -> str:
        return f'<"{camelCase(self.name)}"={self.value}>'

    def request_value_update(self, value) -> None:
        self.value = value
        self.sync_evt.set()


def camelCase(st):
    return ''.join(x for x in st.title() if x.isalnum())
