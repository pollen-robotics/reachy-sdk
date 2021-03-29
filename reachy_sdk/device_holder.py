"""Generic Device Holder."""

from typing import ItemsView, KeysView, List, Union, ValuesView

from .fan import Fan
from .force_sensor import ForceSensor
from .joint import Joint


Device = Union[Fan, ForceSensor, Joint]


class DeviceHolder:
    """Device holder class which contains a specific type of device (eg. Fan, ForceSensor, etc).

    Its only purpose is to not overcrowed the ReachySDK class with all devices.
    """

    def __init__(self, device_list: List[Device]) -> None:
        """Set up devices. Each given device is an attribute of the class."""
        self._devices = {dev.name: dev for dev in device_list}
        for dev in device_list:
            setattr(self, dev.name, dev)

    def __repr__(self) -> str:
        """Clean representation of all the devices in the class."""
        s = '\n\t'.join([str(f) for f in self._devices.values()])
        return f'''<Holder\n\t{
            s
        }\n>'''

    def items(self) -> ItemsView[str, Device]:
        """Get a dict item view of the contained devices."""
        return self._devices.items()

    def keys(self) -> KeysView[str]:
        """Get a dict key view of the contained devices."""
        return self._devices.keys()

    def values(self) -> ValuesView[Device]:
        """Get a dict value view of the contained devices."""
        return self._devices.values()

    def _get_device_from_id(self, device_id):
        if device_id.HasField('uid'):
            for device in self._devices.values():
                if device.uid == device_id.uid:
                    return device
        else:
            for device in self._devices.values():
                if device.name == device_id.name:
                    return device
        raise IndexError
