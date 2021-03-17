"""
Define a Register class and meta-class to handle in background.

Handles:
- whether a register is RO or RW
- the conversion from internal representation (usually a google protobuf value) to the external one (base Python type)
- store of the actual value in the associated instance of the meta-class (Joint in our case).

"""
from typing import Any, Callable, Optional, Tuple, Type

from google.protobuf.wrappers_pb2 import BoolValue, FloatValue, UInt32Value

from reachy_sdk_api.joint_pb2 import ComplianceMarginSlope, PIDGains


class Register:
    """Register class to expose the stored value in a more simple way.

    The internal representation is directly the gRPC object deserialized.
    As this is not convenient to work with, we convert it to base python type (int, tuple, float, etc).

    The register also checks the access right (RO or RW).

    The synchronisation with the gRPC messages is actually done by the Joint.
    """

    label: str = ''

    def __init__(self,
                 readonly: bool,
                 type: Type[Any],
                 conversion: Optional[Tuple[Callable[[Any], Any], Callable[[Any], Any]]] = None,
                 ) -> None:
        """Set up the register."""
        self.readonly = readonly
        self.internal_class = type

        self.cvt_to_internal: Optional[Callable[[Any], Any]] = None
        self.cvt_to_external: Optional[Callable[[Any], Any]] = None
        if conversion is not None:
            self.cvt_to_internal, self.cvt_to_external = conversion

    def __get__(self, instance, owner):
        """Unwrap, optionally convert the internal value and return it."""
        value = self.unwrapped_value(instance[self.label])
        if self.cvt_to_external is not None:
            value = self.cvt_to_external(value)
        return value

    def __set__(self, instance, value):
        """Convert (if needed) the given value, wrap it and store it internally."""
        if self.readonly:
            raise AttributeError("can't set attribute")

        if self.cvt_to_internal is not None:
            value = self.cvt_to_internal(value)

        instance[self.label] = self.wrapped_value(value)

    def unwrapped_value(self, value: Any) -> Any:
        """Unwrap the internal value to a more simple one."""
        if self.internal_class in (BoolValue, FloatValue, UInt32Value):
            return value.value

        # Circumvent https://github.com/grpc/grpc/issues/18139
        if self.internal_class.__name__ == 'PIDValue':
            if value.HasField('pid'):
                return (value.pid.p, value.pid.i, value.pid.d)
            elif value.HasField('compliance'):
                return (
                    value.compliance.cw_compliance_margin, value.compliance.ccw_compliance_margin,
                    value.compliance.cw_compliance_slope, value.compliance.ccw_compliance_slope,
                )
            else:
                raise ValueError('Empty PIDValue received!')

        return value

    def wrapped_value(self, value: Any) -> Any:
        """Wrap the simple Python value to the corresponding gRPC one."""
        if self.internal_class in (BoolValue, FloatValue, UInt32Value):
            return self.internal_class(value=value)

        elif self.internal_class.__name__ == 'PIDValue':
            if len(value) == 3:
                return self.internal_class(pid=PIDGains(p=value[0], i=value[1], d=value[2]))
            elif len(value) == 4:
                return self.internal_class(compliance=ComplianceMarginSlope(
                    cw_compliance_margin=value[0], ccw_compliance_margin=value[1],
                    cw_compliance_slope=value[2], ccw_compliance_slope=value[3],
                ))
            else:
                raise ValueError('Empty PIDValue received!')

        return value


class MetaRegister(type):
    """MetaClass used to facilitate the registration of the Register in the Joint."""

    def __new__(cls, name, bases, attrs):
        """Register the label of the register as it is used by the Joint to store its value using it as key."""
        for field, value in attrs.items():
            if isinstance(value, Register):
                value.label = field
        return super().__new__(cls, name, bases, attrs)
