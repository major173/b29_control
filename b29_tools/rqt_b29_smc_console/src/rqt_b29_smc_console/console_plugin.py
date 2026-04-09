import os
import time
from importlib import import_module

import rospkg

from .models import FieldValueType
from .trace_model import TraceModel

try:
    from rqt_gui_py.plugin import Plugin
except ImportError:  # pragma: no cover - fallback for smoke imports without full ROS GUI context
    try:
        from qt_gui.plugin import Plugin
    except ImportError:  # pragma: no cover
        class Plugin(object):
            def __init__(self, context):
                self._context = context

            def setObjectName(self, name):
                self._object_name = name

try:
    from python_qt_binding import loadUi
    from python_qt_binding.QtCore import QObject, Signal
    from python_qt_binding.QtGui import QColor
    from python_qt_binding.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QGridLayout,
        QGroupBox,
        QLabel,
        QListWidgetItem,
        QLineEdit,
        QPushButton,
        QStackedWidget,
        QWidget,
    )
except ImportError:  # pragma: no cover - fallback for non-catkin smoke tests
    from PyQt5 import uic
    from PyQt5.QtCore import QObject, pyqtSignal as Signal
    from PyQt5.QtGui import QColor
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QGridLayout,
        QGroupBox,
        QLabel,
        QListWidgetItem,
        QLineEdit,
        QPushButton,
        QStackedWidget,
        QWidget,
    )

    def loadUi(ui_file, baseinstance=None, custom_widgets=None):
        return uic.loadUi(ui_file, baseinstance=baseinstance)


_PACKAGE_NAME = 'rqt_b29_smc_console'
_UI_FILENAME = 'console.ui'
DEFAULT_NAMESPACE = '/b29_controller/b29_smc_auto_controller'

_ACTION_PULSE = 'pulse'
_ACTION_LATCH = 'latch'
_ACTION_CLEAR = 'clear'
_ACTION_ITEMS = (
    ('触发一次', _ACTION_PULSE),
    ('持续覆盖', _ACTION_LATCH),
    ('取消覆盖', _ACTION_CLEAR),
)
_TRACE_HIGHLIGHTS = {
    'SafeStop': QColor('#fde8e8'),
    'CommsLoss': QColor('#fff4cc'),
}


def _default_runtime_importer():
    field_registry_module = import_module('.field_registry', __package__)
    topic_facade_module = import_module('.topic_facade', __package__)
    workflows_module = import_module('.workflows', __package__)
    return {
        'build_debug_override_registry': field_registry_module.build_debug_override_registry,
        'build_sensor_input_registry': field_registry_module.build_sensor_input_registry,
        'get_field_descriptor': field_registry_module.get_field_descriptor,
        'topic_facade_class': topic_facade_module.TopicFacade,
        'workflow_runner_class': workflows_module.WorkflowRunner,
        'base_ready_sensor_payload': workflows_module.BASE_READY_SENSOR_PAYLOAD,
        'build_default_workflows': workflows_module.build_default_workflows,
    }


def _source_ui_file():
    package_root = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    return os.path.join(package_root, 'resource', _UI_FILENAME)


def _package_ui_file():
    package_root = rospkg.RosPack().get_path(_PACKAGE_NAME)
    return os.path.join(package_root, 'resource', _UI_FILENAME)


def _resolve_ui_file():
    tried_paths = []
    for candidate_factory in (_source_ui_file, _package_ui_file):
        try:
            candidate = candidate_factory()
        except Exception as exc:
            tried_paths.append(str(exc))
            continue
        tried_paths.append(candidate)
        if os.path.exists(candidate):
            return candidate
    raise RuntimeError(
        'Unable to locate UI file for {package}. Tried: {paths}'.format(
            package=_PACKAGE_NAME,
            paths=', '.join(tried_paths),
        )
    )


def build_override_choices(registry):
    return [
        {
            'field_name': descriptor.name,
            'text': '{group} / {label}'.format(group=descriptor.group, label=descriptor.label),
            'group': descriptor.group,
            'label': descriptor.label,
            'value_type': descriptor.value_type.value,
            'supports_pulse': descriptor.supports_pulse,
            'enum_options': tuple(descriptor.enum_options),
        }
        for descriptor in registry
    ]


def build_workflow_choices(workflows_map):
    return [{'name': workflow_name, 'text': workflow_name} for workflow_name in workflows_map.keys()]


def format_mask_preview(active_values, registry):
    active_field_names = []
    mask = 0
    for descriptor in registry:
        if descriptor.name not in active_values or descriptor.bit is None:
            continue
        mask |= descriptor.bit
        active_field_names.append(descriptor.name)
    summary = ', '.join(active_field_names) if active_field_names else '无'
    return '十进制: {mask} | 十六进制: 0x{mask:04X} | 字段: {summary}'.format(mask=mask, summary=summary)


def _build_field_map(registry):
    return {descriptor.name: descriptor for descriptor in registry}


def _lookup_descriptor(registry, field_name):
    for descriptor in registry:
        if descriptor.name == field_name:
            return descriptor
    return None


def _fallback_sensor_preset(sensor_registry):
    preset = {}
    for descriptor in sensor_registry:
        preset[descriptor.name] = descriptor.default_value
    return preset


def _comms_loss_preset(sensor_registry):
    payload = dict(_fallback_sensor_preset(sensor_registry))
    payload.update(
        {
            'lower_alive': False,
            'imu_ready': False,
            'grip_confirmed': False,
        }
    )
    return payload


def _obstacle_detected_preset(sensor_registry, base_ready_sensor_payload):
    payload = dict(base_ready_sensor_payload or _fallback_sensor_preset(sensor_registry))
    obstacle_type_descriptor = _lookup_descriptor(sensor_registry, 'obstacle_type')
    obstacle_value = 1
    if obstacle_type_descriptor is not None:
        if len(obstacle_type_descriptor.enum_options) > 1:
            obstacle_value = obstacle_type_descriptor.enum_options[1].value
        else:
            obstacle_value = obstacle_type_descriptor.default_value
    payload.update(
        {
            'obstacle_detected': True,
            'obstacle_type': obstacle_value,
            'classification_stable': True,
            'range_to_obstacle': 0.35,
        }
    )
    return payload


def _format_reason(entry):
    details = []
    if entry.transition_reason:
        details.append('transition: {value}'.format(value=entry.transition_reason))
    if entry.command_reason:
        details.append('command: {value}'.format(value=entry.command_reason))
    return ' | '.join(details) if details else '-'


class _TraceSignalProxy(QObject):
    received = Signal(object)


class _NullTopicFacade(object):
    def publish_sensor_input(self, values):
        return values

    def latch_override(self, field_name, value):
        return field_name, value

    def clear_override(self, field_name):
        return field_name

    def pulse_override(self, field_name, value=True):
        return field_name, value


class _NullWorkflowRunner(object):
    def run(self, workflow_name):
        return workflow_name


class B29SmcConsolePlugin(Plugin):
    def __init__(
        self,
        context,
        namespace=DEFAULT_NAMESPACE,
        topic_facade_factory=None,
        workflow_runner_factory=None,
        sensor_registry=None,
        override_registry=None,
        workflows_map=None,
        trace_model=None,
        runtime_importer=None,
    ):
        super(B29SmcConsolePlugin, self).__init__(context)
        self.setObjectName('B29SmcConsolePlugin')
        self._widget = QWidget()
        loadUi(_resolve_ui_file(), self._widget)
        self._widget.setObjectName('B29SmcConsoleWidget')
        self._widget.setWindowTitle('B29 SMC Console')

        self._runtime_note = ''
        runtime = {}
        try:
            runtime = (runtime_importer or _default_runtime_importer)()
        except Exception as exc:
            self._runtime_note = str(exc)

        self._namespace = namespace or DEFAULT_NAMESPACE
        self._sensor_registry = tuple(
            sensor_registry or (runtime.get('build_sensor_input_registry', lambda: ())())
        )
        self._override_registry = tuple(
            override_registry or (runtime.get('build_debug_override_registry', lambda: ())())
        )
        self._workflows_map = workflows_map or (runtime.get('build_default_workflows', lambda: {})())
        self._base_ready_sensor_payload = dict(runtime.get('base_ready_sensor_payload', {}))
        self._trace_model = trace_model or TraceModel()
        self._override_active_values = {}
        self._sensor_widgets = {}
        self._override_descriptor_map = _build_field_map(self._override_registry)
        self._override_choices = build_override_choices(self._override_registry)
        self._trace_signal_proxy = _TraceSignalProxy()
        self._trace_signal_proxy.received.connect(self._apply_trace_message)

        self._bind_static_widgets()
        self._build_sensor_inputs()
        self._configure_override_controls()
        self._configure_workflow_controls()
        self._bind_actions()
        self._refresh_overview(None)

        topic_facade_factory = topic_facade_factory or (
            lambda namespace, trace_callback: runtime['topic_facade_class'](namespace, trace_callback=trace_callback)
        )
        workflow_runner_factory = workflow_runner_factory or (
            lambda topic_facade, wait_for_state: runtime['workflow_runner_class'](topic_facade, wait_for_state)
        )
        try:
            self._topic_facade = topic_facade_factory(self._namespace, self._handle_trace_message)
            self._connection_note = self._runtime_note
        except Exception as exc:  # pragma: no cover - only exercised without ROS runtime
            self._topic_facade = _NullTopicFacade()
            failure_note = str(exc)
            self._connection_note = ', '.join(item for item in (self._runtime_note, failure_note) if item)
        try:
            self._workflow_runner = workflow_runner_factory(self._topic_facade, self._wait_for_state)
        except Exception:
            self._workflow_runner = _NullWorkflowRunner()
        if self._connection_note:
            self.overrideActionHintLabel.setText('离线模式: 运行时依赖缺失或 ROS 未就绪。{message}'.format(message=self._connection_note))

        add_widget = getattr(context, 'add_widget', None)
        if callable(add_widget):
            add_widget(self._widget)

    def _bind_static_widgets(self):
        self.namespaceValueLineEdit = self._widget.findChild(QLineEdit, 'namespaceValueLineEdit')
        self.overviewNamespaceValueLabel = self._widget.findChild(QLabel, 'overviewNamespaceValueLabel')
        self.currentStateValueLabel = self._widget.findChild(QLabel, 'currentStateValueLabel')
        self.previousStateValueLabel = self._widget.findChild(QLabel, 'previousStateValueLabel')
        self.lastEventValueLabel = self._widget.findChild(QLabel, 'lastEventValueLabel')
        self.outputModeValueLabel = self._widget.findChild(QLabel, 'outputModeValueLabel')
        self.reasonValueLabel = self._widget.findChild(QLabel, 'reasonValueLabel')
        self.overrideFieldComboBox = self._widget.findChild(QComboBox, 'overrideFieldComboBox')
        self.overrideActionComboBox = self._widget.findChild(QComboBox, 'overrideActionComboBox')
        self.overrideValueStackedWidget = self._widget.findChild(QStackedWidget, 'overrideValueStackedWidget')
        self.overrideBoolValueCheckBox = self._widget.findChild(QCheckBox, 'overrideBoolValueCheckBox')
        self.overrideEnumValueComboBox = self._widget.findChild(QComboBox, 'overrideEnumValueComboBox')
        self.overrideFloatValueSpinBox = self._widget.findChild(QDoubleSpinBox, 'overrideFloatValueSpinBox')
        self.overrideActionHintLabel = self._widget.findChild(QLabel, 'overrideActionHintLabel')
        self.overrideMaskPreviewLabel = self._widget.findChild(QLabel, 'overrideMaskPreviewLabel')
        self.workflowComboBox = self._widget.findChild(QComboBox, 'workflowComboBox')
        self.workflowStatusLabel = self._widget.findChild(QLabel, 'workflowStatusLabel')
        self.traceListWidget = self._widget.findChild(QWidget, 'traceListWidget')
        self.publishSensorInputButton = self._widget.findChild(QPushButton, 'publishSensorInputButton')
        self.sendOverrideButton = self._widget.findChild(QPushButton, 'sendOverrideButton')
        self.runWorkflowButton = self._widget.findChild(QPushButton, 'runWorkflowButton')
        self.presetBaseReadyButton = self._widget.findChild(QPushButton, 'presetBaseReadyButton')
        self.presetCommsLossButton = self._widget.findChild(QPushButton, 'presetCommsLossButton')
        self.presetObstacleButton = self._widget.findChild(QPushButton, 'presetObstacleButton')
        self.presetClearButton = self._widget.findChild(QPushButton, 'presetClearButton')
        self.sensorGroupsLayout = self._widget.findChild(QWidget, 'sensorScrollAreaContents').layout()
        self.namespaceValueLineEdit.setText(self._namespace)

    def _build_sensor_inputs(self):
        grouped_descriptors = {}
        for descriptor in self._sensor_registry:
            grouped_descriptors.setdefault(descriptor.group, []).append(descriptor)

        for group_name, descriptors in grouped_descriptors.items():
            group_box = QGroupBox(group_name)
            group_layout = QGridLayout(group_box)
            group_layout.setHorizontalSpacing(12)
            group_layout.setVerticalSpacing(8)
            for row, descriptor in enumerate(descriptors):
                label = QLabel(descriptor.label)
                control = self._create_input_widget(descriptor, 'sensorInput__{name}'.format(name=descriptor.name))
                group_layout.addWidget(label, row, 0)
                group_layout.addWidget(control, row, 1)
                self._sensor_widgets[descriptor.name] = control
            self.sensorGroupsLayout.addWidget(group_box)
        self.sensorGroupsLayout.addStretch(1)

    def _create_input_widget(self, descriptor, object_name):
        if descriptor.value_type == FieldValueType.BOOL:
            widget = QCheckBox()
            widget.setChecked(bool(descriptor.default_value))
        elif descriptor.value_type == FieldValueType.ENUM:
            widget = QComboBox()
            for option in descriptor.enum_options:
                widget.addItem(option.label, option.value)
            self._set_combo_to_value(widget, descriptor.default_value)
        else:
            widget = QDoubleSpinBox()
            widget.setDecimals(3)
            widget.setMinimum(-9999.0)
            widget.setMaximum(9999.0)
            widget.setValue(float(descriptor.default_value or 0.0))
        widget.setObjectName(object_name)
        return widget

    def _configure_override_controls(self):
        for choice in self._override_choices:
            self.overrideFieldComboBox.addItem(choice['text'], choice['field_name'])

        for text, value in _ACTION_ITEMS:
            self.overrideActionComboBox.addItem(text, value)

        self.overrideFloatValueSpinBox.setSingleStep(0.05)
        self._sync_override_editor()

    def _configure_workflow_controls(self):
        for choice in build_workflow_choices(self._workflows_map):
            self.workflowComboBox.addItem(choice['text'], choice['name'])

    def _bind_actions(self):
        self.presetBaseReadyButton.clicked.connect(
            lambda: self._apply_sensor_preset(dict(self._base_ready_sensor_payload or _fallback_sensor_preset(self._sensor_registry)))
        )
        self.presetCommsLossButton.clicked.connect(lambda: self._apply_sensor_preset(_comms_loss_preset(self._sensor_registry)))
        self.presetObstacleButton.clicked.connect(
            lambda: self._apply_sensor_preset(_obstacle_detected_preset(self._sensor_registry, self._base_ready_sensor_payload))
        )
        self.presetClearButton.clicked.connect(lambda: self._apply_sensor_preset(_fallback_sensor_preset(self._sensor_registry)))
        self.publishSensorInputButton.clicked.connect(self._publish_sensor_input)
        self.overrideFieldComboBox.currentIndexChanged.connect(self._sync_override_editor)
        self.overrideActionComboBox.currentIndexChanged.connect(self._sync_override_editor)
        self.overrideBoolValueCheckBox.toggled.connect(self._refresh_override_preview)
        self.overrideEnumValueComboBox.currentIndexChanged.connect(self._refresh_override_preview)
        self.overrideFloatValueSpinBox.valueChanged.connect(self._refresh_override_preview)
        self.sendOverrideButton.clicked.connect(self._send_override)
        self.runWorkflowButton.clicked.connect(self._run_workflow)

    def _apply_sensor_preset(self, payload):
        for descriptor in self._sensor_registry:
            self._set_widget_value(self._sensor_widgets[descriptor.name], descriptor, payload.get(descriptor.name, descriptor.default_value))

    def _publish_sensor_input(self):
        self._topic_facade.publish_sensor_input(self._collect_sensor_values())

    def _configure_override_value_widget(self, descriptor):
        current_value = self._override_active_values.get(descriptor.name, descriptor.default_value)
        if descriptor.value_type == FieldValueType.BOOL:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideBoolValueCheckBox.parentWidget())
            self.overrideBoolValueCheckBox.setChecked(bool(self._override_active_values.get(descriptor.name, True)))
        elif descriptor.value_type == FieldValueType.ENUM:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideEnumValueComboBox.parentWidget())
            self.overrideEnumValueComboBox.blockSignals(True)
            self.overrideEnumValueComboBox.clear()
            for option in descriptor.enum_options:
                self.overrideEnumValueComboBox.addItem(option.label, option.value)
            self._set_combo_to_value(self.overrideEnumValueComboBox, current_value)
            self.overrideEnumValueComboBox.blockSignals(False)
        else:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideFloatValueSpinBox.parentWidget())
            self.overrideFloatValueSpinBox.setValue(float(current_value or 0.0))

    def _sync_override_editor(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return

        self._configure_override_value_widget(descriptor)
        pulse_index = self.overrideActionComboBox.findData(_ACTION_PULSE)
        pulse_item = self.overrideActionComboBox.model().item(pulse_index) if pulse_index >= 0 else None
        if pulse_item is not None:
            pulse_item.setEnabled(descriptor.supports_pulse)
        if not descriptor.supports_pulse and self._current_override_action() == _ACTION_PULSE:
            latch_index = self.overrideActionComboBox.findData(_ACTION_LATCH)
            self.overrideActionComboBox.setCurrentIndex(latch_index)

        if descriptor.supports_pulse:
            self.overrideActionHintLabel.setText('当前字段支持一次触发或持续覆盖。')
        else:
            self.overrideActionHintLabel.setText('当前字段不支持“触发一次”，将使用持续覆盖或取消覆盖。')
        self._refresh_override_preview()

    def _refresh_override_preview(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            self.overrideMaskPreviewLabel.setText('-')
            return

        projected = dict(self._override_active_values)
        action = self._current_override_action()
        if action in (_ACTION_PULSE, _ACTION_LATCH):
            projected[descriptor.name] = self._read_widget_value(self._current_override_value_widget(), descriptor)
        elif action == _ACTION_CLEAR:
            projected.pop(descriptor.name, None)
        self.overrideMaskPreviewLabel.setText(format_mask_preview(projected, self._override_registry))

    def _send_override(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return

        action = self._current_override_action()
        value = self._read_widget_value(self._current_override_value_widget(), descriptor)
        if action == _ACTION_PULSE:
            self._topic_facade.pulse_override(descriptor.name, value)
        elif action == _ACTION_LATCH:
            self._override_active_values[descriptor.name] = value
            self._topic_facade.latch_override(descriptor.name, value)
        else:
            self._override_active_values.pop(descriptor.name, None)
            self._topic_facade.clear_override(descriptor.name)
        self._refresh_override_preview()

    def _run_workflow(self):
        workflow_name = self.workflowComboBox.currentData()
        if workflow_name:
            self.workflowStatusLabel.setText('已触发 workflow: {name}'.format(name=workflow_name))
            try:
                self._workflow_runner.run(workflow_name)
            except Exception as exc:
                self.workflowStatusLabel.setText('Workflow 执行失败: {message}'.format(message=str(exc)))
            else:
                self.workflowStatusLabel.setText('Workflow 已执行: {name}，等待状态结果。'.format(name=workflow_name))

    def _collect_sensor_values(self):
        values = {}
        for descriptor in self._sensor_registry:
            values[descriptor.name] = self._read_widget_value(self._sensor_widgets[descriptor.name], descriptor)
        return values

    def _current_override_descriptor(self):
        field_name = self.overrideFieldComboBox.currentData()
        if not field_name:
            return None
        return self._override_descriptor_map.get(field_name)

    def _current_override_action(self):
        return self.overrideActionComboBox.currentData()

    def _current_override_value_widget(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return self.overrideBoolValueCheckBox
        if descriptor.value_type == FieldValueType.BOOL:
            return self.overrideBoolValueCheckBox
        if descriptor.value_type == FieldValueType.ENUM:
            return self.overrideEnumValueComboBox
        return self.overrideFloatValueSpinBox

    def _set_widget_value(self, widget, descriptor, value):
        if descriptor.value_type == FieldValueType.BOOL:
            widget.setChecked(bool(value))
        elif descriptor.value_type == FieldValueType.ENUM:
            self._set_combo_to_value(widget, value)
        else:
            widget.setValue(float(value or 0.0))

    @staticmethod
    def _set_combo_to_value(combo_box, value):
        index = combo_box.findData(value)
        combo_box.setCurrentIndex(index if index >= 0 else 0)

    @staticmethod
    def _read_widget_value(widget, descriptor):
        if descriptor.value_type == FieldValueType.BOOL:
            return widget.isChecked()
        if descriptor.value_type == FieldValueType.ENUM:
            return widget.currentData()
        return float(widget.value())

    def _refresh_overview(self, entry):
        self.overviewNamespaceValueLabel.setText(self._namespace)
        if entry is None:
            self.currentStateValueLabel.setText('-')
            self.previousStateValueLabel.setText('-')
            self.lastEventValueLabel.setText('-')
            self.outputModeValueLabel.setText('-')
            self.reasonValueLabel.setText('-')
            return

        self.currentStateValueLabel.setText(entry.current_state or '-')
        self.previousStateValueLabel.setText(entry.previous_state or '-')
        self.lastEventValueLabel.setText(entry.last_event or '-')
        self.outputModeValueLabel.setText(entry.output_mode or '-')
        self.reasonValueLabel.setText(_format_reason(entry))

    def _handle_trace_message(self, message):
        self._trace_signal_proxy.received.emit(message)

    def _apply_trace_message(self, message):
        entry = self._trace_model.update(message)
        self._refresh_overview(entry)
        self._refresh_trace_history()

    def _refresh_trace_history(self):
        self.traceListWidget.clear()
        for entry in self._trace_model.history():
            item = QListWidgetItem(
                '{current} | {event} | prev: {previous} | mode: {mode} | {reason}'.format(
                    current=entry.current_state or '-',
                    event=entry.last_event or '-',
                    previous=entry.previous_state or '-',
                    mode=entry.output_mode or '-',
                    reason=_format_reason(entry),
                )
            )
            if entry.current_state in _TRACE_HIGHLIGHTS:
                item.setBackground(_TRACE_HIGHLIGHTS[entry.current_state])
            self.traceListWidget.addItem(item)

    def _wait_for_state(self, expected_state, timeout_sec):
        deadline = time.time() + float(timeout_sec or 0.0)
        while time.time() <= deadline:
            latest = self._trace_model.latest()
            if latest and latest.current_state == expected_state:
                return True
            QApplication.processEvents()
            time.sleep(0.02)
        return False
