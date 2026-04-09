from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional, Tuple


class MessageKind(Enum):
    DEBUG_OVERRIDE = 'debug_override'
    SENSOR_INPUT = 'sensor_input'


class FieldValueType(Enum):
    BOOL = 'bool'
    ENUM = 'enum'
    FLOAT = 'float'


@dataclass(frozen=True)
class EnumOption:
    value: Any
    label: str


@dataclass(frozen=True)
class FieldDescriptor:
    name: str
    label: str
    group: str
    message_kind: MessageKind
    value_type: FieldValueType
    default_value: Any = None
    bit: Optional[int] = None
    supports_pulse: bool = False
    enum_options: Tuple[EnumOption, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class TraceEntry:
    current_state: str = ''
    previous_state: str = ''
    last_event: str = ''
    transition_reason: str = ''
    command_reason: str = ''
    output_mode: str = ''


@dataclass(frozen=True)
class WorkflowStep:
    action: str
    description: str
    payload: Any
    expected_state: str = ''
    timeout_sec: float = 0.0
