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
    gravity_compensation_mode: int = 0
    obstacle_crossing_stage: str = ''
    obstacle_crossing_side: str = ''
    disconnect_step: str = ''
    planner_control_active: bool = False
    remote_control_active: bool = False
    software_emergency_stop_latched: bool = False


@dataclass(frozen=True)
class TraceHistoryEntry:
    entry: TraceEntry
    repeat_count: int = 1
    duration_sec: float = 0.0
    first_seen_sec: float = 0.0
    last_seen_sec: float = 0.0


@dataclass(frozen=True)
class TraceUpdateDelta:
    latest: TraceEntry
    raw_entry: TraceHistoryEntry
    raw_appended: bool
    key_entry: TraceHistoryEntry
    key_appended: bool


@dataclass(frozen=True)
class WorkflowStep:
    action: str
    description: str
    payload: Any
    expected_state: str = ''
    timeout_sec: float = 0.0
