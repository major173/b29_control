import threading
from typing import Callable, Dict, Optional, Tuple

try:
    import rospy
except ImportError:  # pragma: no cover - allows local smoke imports without ROS runtime
    class _FallbackTime:
        @staticmethod
        def now():
            return None

    class _FallbackRospy:
        Time = _FallbackTime

        @staticmethod
        def Publisher(*_args, **_kwargs):
            raise RuntimeError('rospy.Publisher is unavailable in this environment')

        @staticmethod
        def Subscriber(*_args, **_kwargs):
            raise RuntimeError('rospy.Subscriber is unavailable in this environment')

        @staticmethod
        def ServiceProxy(*_args, **_kwargs):
            raise RuntimeError('rospy.ServiceProxy is unavailable in this environment')

    rospy = _FallbackRospy()

from b29_smc_auto_controller.msg import AutoDebugOverride, AutoSensorInput, AutoStateTrace
from std_srvs.srv import Trigger

from .field_registry import build_debug_override_registry, build_sensor_input_registry, get_field_descriptor
from .models import MessageKind

_DEBUG_FIELD_NAMES = tuple(item.name for item in build_debug_override_registry())
_SENSOR_FIELD_NAMES = tuple(item.name for item in build_sensor_input_registry())


def _namespace_topic(namespace: str, suffix: str) -> str:
    base = namespace.rstrip('/')
    if not base:
        return f'/{suffix}'
    return f'{base}/{suffix}'


def _stamp_message(message) -> None:
    if hasattr(message, 'header'):
        message.header.stamp = rospy.Time.now()


def compose_sensor_input_message(values: Dict[str, object]) -> AutoSensorInput:
    message = AutoSensorInput()
    for field_name in _SENSOR_FIELD_NAMES:
        if field_name in values:
            setattr(message, field_name, values[field_name])
    _stamp_message(message)
    return message


def compose_override_message(values: Dict[str, object]) -> AutoDebugOverride:
    message = AutoDebugOverride()
    mask = 0
    for field_name in _DEBUG_FIELD_NAMES:
        if field_name not in values:
            continue
        descriptor = get_field_descriptor(field_name, MessageKind.DEBUG_OVERRIDE)
        if descriptor.bit is None:
            continue
        setattr(message, field_name, values[field_name])
        mask |= descriptor.bit
    message.enabled = bool(mask)
    message.field_mask = mask
    _stamp_message(message)
    return message


def _ensure_pulse_support(field_name: str):
    try:
        descriptor = get_field_descriptor(field_name, MessageKind.DEBUG_OVERRIDE)
    except KeyError as exc:
        raise ValueError(f'Unknown debug override field: {field_name}') from exc
    if not descriptor.supports_pulse:
        raise ValueError(f'Field {field_name} does not support pulse')
    return descriptor


class OverrideComposer:
    def __init__(self):
        self._latched_values: Dict[str, object] = {}

    def latch(self, field_name: str, value: object) -> AutoDebugOverride:
        self._latched_values[field_name] = value
        return compose_override_message(self._latched_values)

    def latch_many(self, values: Dict[str, object]) -> AutoDebugOverride:
        self._latched_values.update(values)
        return compose_override_message(self._latched_values)

    def clear(self, field_name: str) -> AutoDebugOverride:
        self._latched_values.pop(field_name, None)
        return compose_override_message(self._latched_values)

    def pulse(self, field_name: str, value: object = True) -> Tuple[AutoDebugOverride, AutoDebugOverride]:
        _ensure_pulse_support(field_name)
        pressed_values = dict(self._latched_values)
        pressed_values[field_name] = value
        pressed = compose_override_message(pressed_values)
        released = compose_override_message(self._latched_values)
        return pressed, released


class TopicFacade:
    def __init__(
        self,
        namespace: str,
        trace_callback: Optional[Callable[[AutoStateTrace], None]] = None,
        publisher_factory: Callable = None,
        subscriber_factory: Callable = None,
        service_proxy_factory: Callable = None,
        release_scheduler: Callable[[float, Callable[[], None]], object] = None,
        pulse_hold_sec: float = 0.05,
    ):
        self._namespace = namespace
        self._composer = OverrideComposer()
        self._trace_callback = trace_callback or (lambda _message: None)
        self._publisher_factory = publisher_factory or rospy.Publisher
        self._subscriber_factory = subscriber_factory or rospy.Subscriber
        self._service_proxy_factory = service_proxy_factory or rospy.ServiceProxy
        self._release_scheduler = release_scheduler or self._default_release_scheduler
        self._pulse_hold_sec = float(pulse_hold_sec)
        self._pending_release_handles = []
        self._sensor_publisher = self._publisher_factory(
            _namespace_topic(namespace, 'sensor_input'),
            AutoSensorInput,
            queue_size=1,
        )
        self._override_publisher = self._publisher_factory(
            _namespace_topic(namespace, 'debug_override'),
            AutoDebugOverride,
            queue_size=1,
        )
        self._state_trace_subscriber = self._subscriber_factory(
            _namespace_topic(namespace, 'state_trace'),
            AutoStateTrace,
            self._trace_callback,
        )
        self._planner_release_service = self._service_proxy_factory(
            _namespace_topic(namespace, 'planner_release'), Trigger
        )
        self._software_emergency_stop_service = self._service_proxy_factory(
            _namespace_topic(namespace, 'software_emergency_stop'), Trigger
        )
        self._manual_reset_service = self._service_proxy_factory(
            _namespace_topic(namespace, 'manual_reset'), Trigger
        )

    @staticmethod
    def _default_release_scheduler(delay_sec: float, callback: Callable[[], None]):
        timer = threading.Timer(delay_sec, callback)
        timer.daemon = True
        timer.start()
        return timer

    def _schedule_release_publish(self, released: AutoDebugOverride) -> None:
        handle_box = {}

        def publish_released():
            try:
                self._override_publisher.publish(released)
            finally:
                handle = handle_box.get('handle')
                if handle in self._pending_release_handles:
                    self._pending_release_handles.remove(handle)

        handle = self._release_scheduler(self._pulse_hold_sec, publish_released)
        handle_box['handle'] = handle
        if handle is not None:
            self._pending_release_handles.append(handle)

    def publish_sensor_input(self, values: Dict[str, object]) -> AutoSensorInput:
        message = compose_sensor_input_message(values)
        self._sensor_publisher.publish(message)
        return message

    def latch_override(self, field_name: str, value: object) -> AutoDebugOverride:
        message = self._composer.latch(field_name, value)
        self._override_publisher.publish(message)
        return message

    def publish_obstacle_override(self, detected: bool, distance: float) -> AutoDebugOverride:
        message = self._composer.latch_many(
            {
                'obstacle_detected': bool(detected),
                'range_to_obstacle': float(distance),
            }
        )
        self._override_publisher.publish(message)
        return message

    def clear_override(self, field_name: str) -> AutoDebugOverride:
        message = self._composer.clear(field_name)
        self._override_publisher.publish(message)
        return message

    def pulse_override(self, field_name: str, value: object = True) -> Tuple[AutoDebugOverride, AutoDebugOverride]:
        pressed, released = self._composer.pulse(field_name, value)
        self._override_publisher.publish(pressed)
        self._schedule_release_publish(released)
        return pressed, released

    def call_planner_release(self):
        return self._planner_release_service()

    def call_software_emergency_stop(self):
        return self._software_emergency_stop_service()

    def call_manual_reset(self):
        return self._manual_reset_service()
