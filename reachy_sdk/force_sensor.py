"""This module define the Force Sensor class."""

from typing import List

from reachy_sdk_api.sensor_pb2 import ForceSensorState


class ForceSensor:
    """The Force sensor class.

    The Force Sensor class is used to store the up-to-date force read by the sensor.
    """

    def __init__(self, name: str, uid: int, state: ForceSensorState) -> None:
        """Set up the force sensor."""
        self._name = name
        self._uid = uid

        self._update_with(state)

    def __repr__(self) -> str:
        """Clean representation of a force sensor state."""
        return f'<ForceSensor name="{self.name}" force="{self.force:.2f}">'

    @property
    def name(self):
        """Get the name of the force sensor."""
        return self._name

    @property
    def uid(self):
        """Get the name of the force sensor."""
        return self._uid

    @property
    def force(self):
        """Get the current force read by the sensor."""
        return self._force

    def _update_with(self, state: ForceSensorState):
        self._force = state.force


class ForceSensors:
    """ForceSensors class.

    Contains each given force sensor.
    """

    def __init__(self, force_sensors_list: List) -> None:
        """Set up the force sensors, each given force sensor is an attribute of the class."""
        self.force_sensors_list = force_sensors_list
        self._setup_force_sensors(force_sensors_list)

    def __repr__(self) -> str:
        """Clean representation of all the force sensors in the class."""
        return '\n'.join([str(f) for f in self.force_sensors_list])

    def _setup_force_sensors(self, force_sensors_list):
        for fs in force_sensors_list:
            setattr(self, fs.name, fs)
