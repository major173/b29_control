from typing import Dict, Iterable, Optional, Sequence, Tuple

from b29_smc_auto_controller.msg import AutoDebugOverride, AutoSensorInput

from .models import EnumOption, FieldDescriptor, FieldValueType, MessageKind

_GROUP_START_RESET = '启动/复位'
_GROUP_BASE = '基础运行'
_GROUP_SAFETY = '安全保护'
_GROUP_OBSTACLE = '障碍相关'
_GROUP_STAGE = '运动阶段'
_GROUP_POST_CHECK = '后检'


def _field(
    *,
    name: str,
    label: str,
    group: str,
    message_kind: MessageKind,
    value_type: FieldValueType,
    default_value=None,
    bit: Optional[int] = None,
    supports_pulse: bool = False,
    enum_options: Iterable[EnumOption] = (),
) -> FieldDescriptor:
    return FieldDescriptor(
        name=name,
        label=label,
        group=group,
        message_kind=message_kind,
        value_type=value_type,
        default_value=default_value,
        bit=bit,
        supports_pulse=supports_pulse,
        enum_options=tuple(enum_options),
    )


def _obstacle_enum_options() -> Tuple[EnumOption, ...]:
    return (
        EnumOption(value=AutoDebugOverride.OBSTACLE_UNKNOWN, label='Unknown'),
        EnumOption(value=AutoDebugOverride.OBSTACLE_LINE_CLAMP, label='Line Clamp'),
        EnumOption(value=AutoDebugOverride.OBSTACLE_DAMPER, label='Damper'),
    )


def _build_debug_override_registry() -> Dict[str, FieldDescriptor]:
    obstacle_options = _obstacle_enum_options()
    return {
        'auto_start_requested': _field(
            name='auto_start_requested',
            label='自动启动请求',
            group=_GROUP_START_RESET,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_AUTO_START_REQUESTED,
            supports_pulse=True,
        ),
        'manual_reset_requested': _field(
            name='manual_reset_requested',
            label='手动复位请求',
            group=_GROUP_START_RESET,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_MANUAL_RESET_REQUESTED,
            supports_pulse=True,
        ),
        'emergency_stop': _field(
            name='emergency_stop',
            label='紧急停止',
            group=_GROUP_START_RESET,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_EMERGENCY_STOP,
            supports_pulse=True,
        ),
        'lower_alive': _field(
            name='lower_alive',
            label='下位机在线',
            group=_GROUP_BASE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_LOWER_ALIVE,
        ),
        'imu_ready': _field(
            name='imu_ready',
            label='IMU就绪',
            group=_GROUP_BASE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_IMU_READY,
        ),
        'posture_ready': _field(
            name='posture_ready',
            label='姿态就绪',
            group=_GROUP_BASE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_POSTURE_READY,
        ),
        'grip_confirmed': _field(
            name='grip_confirmed',
            label='夹持确认',
            group=_GROUP_BASE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_GRIP_CONFIRMED,
        ),
        'joint_fault': _field(
            name='joint_fault',
            label='关节故障',
            group=_GROUP_SAFETY,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_JOINT_FAULT,
        ),
        'grip_fault': _field(
            name='grip_fault',
            label='夹持故障',
            group=_GROUP_SAFETY,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_GRIP_FAULT,
        ),
        'obstacle_detected': _field(
            name='obstacle_detected',
            label='障碍检测',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_OBSTACLE_DETECTED,
        ),
        'obstacle_type': _field(
            name='obstacle_type',
            label='障碍类型',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.ENUM,
            default_value=AutoDebugOverride.OBSTACLE_UNKNOWN,
            bit=AutoDebugOverride.FIELD_OBSTACLE_TYPE,
            enum_options=obstacle_options,
        ),
        'classification_stable': _field(
            name='classification_stable',
            label='分类稳定',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_CLASSIFICATION_STABLE,
        ),
        'range_to_obstacle': _field(
            name='range_to_obstacle',
            label='障碍距离',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.FLOAT,
            default_value=0.0,
            bit=AutoDebugOverride.FIELD_RANGE_TO_OBSTACLE,
        ),
        'at_crossing_position': _field(
            name='at_crossing_position',
            label='位于过障位置',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_AT_CROSSING_POSITION,
        ),
        'crossing_step_done': _field(
            name='crossing_step_done',
            label='过障步骤完成',
            group=_GROUP_STAGE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_CROSSING_STEP_DONE,
        ),
        'crossing_complete': _field(
            name='crossing_complete',
            label='过障完成',
            group=_GROUP_STAGE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_CROSSING_COMPLETE,
        ),
        'post_check_passed': _field(
            name='post_check_passed',
            label='后检通过',
            group=_GROUP_POST_CHECK,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_POST_CHECK_PASSED,
        ),
        'post_check_failed': _field(
            name='post_check_failed',
            label='后检失败',
            group=_GROUP_POST_CHECK,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_POST_CHECK_FAILED,
        ),
        'auto_run_pause': _field(
            name='auto_run_pause',
            label='自动运行暂停',
            group=_GROUP_STAGE,
            message_kind=MessageKind.DEBUG_OVERRIDE,
            value_type=FieldValueType.BOOL,
            default_value=False,
            bit=AutoDebugOverride.FIELD_AUTO_RUN_PAUSE,
            supports_pulse=True,
        ),
    }


def _build_sensor_input_registry() -> Dict[str, FieldDescriptor]:
    obstacle_options = (
        EnumOption(value=AutoSensorInput.OBSTACLE_UNKNOWN, label='Unknown'),
        EnumOption(value=AutoSensorInput.OBSTACLE_LINE_CLAMP, label='Line Clamp'),
        EnumOption(value=AutoSensorInput.OBSTACLE_DAMPER, label='Damper'),
    )
    return {
        'lower_alive': _field(
            name='lower_alive',
            label='下位机在线',
            group=_GROUP_BASE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'imu_ready': _field(
            name='imu_ready',
            label='IMU就绪',
            group=_GROUP_BASE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'grip_confirmed': _field(
            name='grip_confirmed',
            label='夹持确认',
            group=_GROUP_BASE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'joint_fault': _field(
            name='joint_fault',
            label='关节故障',
            group=_GROUP_SAFETY,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'grip_fault': _field(
            name='grip_fault',
            label='夹持故障',
            group=_GROUP_SAFETY,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'obstacle_detected': _field(
            name='obstacle_detected',
            label='障碍检测',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'obstacle_type': _field(
            name='obstacle_type',
            label='障碍类型',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.ENUM,
            default_value=AutoSensorInput.OBSTACLE_UNKNOWN,
            enum_options=obstacle_options,
        ),
        'classification_stable': _field(
            name='classification_stable',
            label='分类稳定',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'range_to_obstacle': _field(
            name='range_to_obstacle',
            label='障碍距离',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.FLOAT,
            default_value=0.0,
        ),
        'at_crossing_position': _field(
            name='at_crossing_position',
            label='位于过障位置',
            group=_GROUP_OBSTACLE,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'post_check_passed': _field(
            name='post_check_passed',
            label='后检通过',
            group=_GROUP_POST_CHECK,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
        'post_check_failed': _field(
            name='post_check_failed',
            label='后检失败',
            group=_GROUP_POST_CHECK,
            message_kind=MessageKind.SENSOR_INPUT,
            value_type=FieldValueType.BOOL,
            default_value=False,
        ),
    }


_DEBUG_OVERRIDE_FIELD_MAP = _build_debug_override_registry()
_SENSOR_INPUT_FIELD_MAP = _build_sensor_input_registry()


def build_debug_override_registry() -> Sequence[FieldDescriptor]:
    return tuple(_DEBUG_OVERRIDE_FIELD_MAP.values())


def build_sensor_input_registry() -> Sequence[FieldDescriptor]:
    return tuple(_SENSOR_INPUT_FIELD_MAP.values())


def get_field_descriptor(field_name: str, message_kind: Optional[MessageKind] = None) -> FieldDescriptor:
    if message_kind == MessageKind.SENSOR_INPUT:
        return _SENSOR_INPUT_FIELD_MAP[field_name]
    if message_kind == MessageKind.DEBUG_OVERRIDE:
        return _DEBUG_OVERRIDE_FIELD_MAP[field_name]

    if field_name in _DEBUG_OVERRIDE_FIELD_MAP:
        return _DEBUG_OVERRIDE_FIELD_MAP[field_name]
    return _SENSOR_INPUT_FIELD_MAP[field_name]
