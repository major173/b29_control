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
    from python_qt_binding.QtCore import QObject, Qt, Signal
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
    from PyQt5.QtCore import QObject, Qt, pyqtSignal as Signal
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
_LANGUAGE_EN = 'en'
_LANGUAGE_ZH = 'zh'
_LANGUAGE_ITEMS = (
    ('English', _LANGUAGE_EN),
    ('中文', _LANGUAGE_ZH),
)

_ACTION_PULSE = 'pulse'
_ACTION_LATCH = 'latch'
_ACTION_CLEAR = 'clear'
_TRACE_MODE_KEY = 'key'
_TRACE_MODE_RAW = 'raw'
_TRACE_HIGHLIGHTS = {
    'SafeStop': QColor('#fde8e8'),
    'CommsLoss': QColor('#fff4cc'),
}
_GROUP_TRANSLATIONS = {
    _LANGUAGE_EN: {
        '启动/复位': 'Start',
        '基础运行': 'Base',
        '安全保护': 'Safety',
        '障碍相关': 'Obstacle',
        '运动阶段': 'Stage',
        '后检': 'Post Check',
    },
    _LANGUAGE_ZH: {
        '启动/复位': '启动/复位',
        '基础运行': '基础运行',
        '安全保护': '安全保护',
        '障碍相关': '障碍相关',
        '运动阶段': '运动阶段',
        '后检': '后检',
    },
}
_ENUM_TRANSLATIONS = {
    _LANGUAGE_EN: {
        'Unknown': 'Unknown',
        'Line Clamp': 'Line Clamp',
        'Damper': 'Damper',
    },
    _LANGUAGE_ZH: {
        'Unknown': '未知',
        'Line Clamp': '线夹',
        'Damper': '阻尼器',
        '未知': '未知',
        '线夹': '线夹',
        '阻尼器': '阻尼器',
    },
}
_TEXTS = {
    _LANGUAGE_EN: {
        'header_subtitle': 'Monitor state, organize sensor input, send override commands, and run the default workflow.',
        'language': 'Language',
        'namespace': 'Namespace',
        'overview_title': 'Overview',
        'overview_namespace': 'namespace',
        'current_state': 'current_state',
        'previous_state': 'previous_state',
        'last_event': 'last_event',
        'output_mode': 'output_mode',
        'reason': 'reason',
        'sensor_inputs_title': 'Sensor Input',
        'preset_base_ready': 'Base Ready',
        'preset_comms_loss': 'Comms Loss',
        'preset_obstacle': 'Obstacle Detected',
        'preset_clear': 'Clear All',
        'publish_sensor_input': 'Publish Sensor Input',
        'override_title': 'Override Composer',
        'override_field': 'Field',
        'override_action': 'Action',
        'override_value': 'Value',
        'override_bool': 'Active',
        'override_action_pulse': 'Pulse once',
        'override_action_latch': 'Latch',
        'override_action_clear': 'Clear override',
        'override_hint_pulse': 'The current field supports pulse once or latch.',
        'override_hint_no_pulse': 'The current field does not support pulse once; latch or clear will be used.',
        'override_send': 'Send Override',
        'workflow_title': 'Workflow',
        'workflow_run': 'Run Workflow',
        'workflow_idle': 'Ready to run workflow.',
        'workflow_sensor_failed': 'Sensor publish failed: {message}',
        'trace_title': 'Trace',
        'trace_mode_label': 'View',
        'trace_mode_key': 'Key Events',
        'trace_mode_raw': 'Raw',
        'trace_pause': 'Pause',
        'trace_resume': 'Resume',
        'trace_clear': 'Clear',
        'trace_repeat': 'x{count} · {duration:.2f}s',
        'workflow_queued': 'Workflow queued: {name}',
        'workflow_failed': 'Workflow failed: {message}',
        'workflow_succeeded': 'Workflow complete: {name}, waiting for state result.',
        'override_publish_failed': 'Override publish failed: {message}',
        'offline_hint': 'Offline mode: runtime dependencies are missing or ROS is not ready. {message}',
        'reason_transition': 'transition',
        'reason_command': 'command',
        'trace_current': 'current',
        'trace_event': 'event',
        'trace_previous': 'prev',
        'trace_mode': 'mode',
        'trace_empty_summary': 'none',
        'trace_mask_decimal': 'Decimal: {mask}',
        'trace_mask_hex': 'Hex: 0x{mask:04X}',
        'trace_mask_fields': 'Fields: {summary}',
    },
    _LANGUAGE_ZH: {
        'header_subtitle': '监控状态、组织传感器输入、发送覆盖命令并执行默认 workflow。',
        'language': '语言',
        'namespace': '命名空间',
        'overview_title': '概览',
        'overview_namespace': 'namespace',
        'current_state': 'current_state',
        'previous_state': 'previous_state',
        'last_event': 'last_event',
        'output_mode': 'output_mode',
        'reason': 'reason',
        'sensor_inputs_title': '传感器输入',
        'preset_base_ready': '基础可启动',
        'preset_comms_loss': '通信丢失',
        'preset_obstacle': '障碍出现',
        'preset_clear': '全部清空',
        'publish_sensor_input': '发布当前传感器输入',
        'override_title': 'Override 组合器',
        'override_field': '字段',
        'override_action': '动作',
        'override_value': '值',
        'override_bool': '激活',
        'override_action_pulse': '触发一次',
        'override_action_latch': '持续覆盖',
        'override_action_clear': '取消覆盖',
        'override_hint_pulse': '当前字段支持一次触发或持续覆盖。',
        'override_hint_no_pulse': '当前字段不支持“触发一次”，将使用持续覆盖或取消覆盖。',
        'override_send': '发送覆盖',
        'workflow_title': '流程',
        'workflow_run': '执行流程',
        'workflow_idle': '等待执行流程。',
        'workflow_sensor_failed': '传感器发布失败: {message}',
        'trace_title': '轨迹',
        'trace_mode_label': '视图',
        'trace_mode_key': '关键事件',
        'trace_mode_raw': '原始流',
        'trace_pause': '暂停',
        'trace_resume': '继续',
        'trace_clear': '清空',
        'trace_repeat': 'x{count} · {duration:.2f}s',
        'workflow_queued': '已触发流程: {name}',
        'workflow_failed': '流程执行失败: {message}',
        'workflow_succeeded': '流程已执行: {name}，等待状态结果。',
        'override_publish_failed': '覆盖发布失败: {message}',
        'offline_hint': '离线模式: 运行时依赖缺失或 ROS 未就绪。{message}',
        'reason_transition': 'transition',
        'reason_command': 'command',
        'trace_current': '当前',
        'trace_event': '事件',
        'trace_previous': '上一个',
        'trace_mode': '模式',
        'trace_empty_summary': '无',
        'trace_mask_decimal': '十进制: {mask}',
        'trace_mask_hex': '十六进制: 0x{mask:04X}',
        'trace_mask_fields': '字段: {summary}',
    },
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


def _language_key(language):
    return language if language in _TEXTS else _LANGUAGE_EN


def _text(language, key, **kwargs):
    template = _TEXTS[_language_key(language)][key]
    return template.format(**kwargs) if kwargs else template


def _translate_group(language, group_name):
    return _GROUP_TRANSLATIONS[_language_key(language)].get(group_name, group_name)


def _translate_enum_label(language, label):
    return _ENUM_TRANSLATIONS[_language_key(language)].get(label, label)


def _translate_field_label(language, descriptor):
    if _language_key(language) == _LANGUAGE_ZH:
        return descriptor.label
    return descriptor.name


def _override_choice_text(language, descriptor):
    return '{group} / {label}'.format(
        group=_translate_group(language, descriptor.group),
        label=_translate_field_label(language, descriptor),
    )


def _translated_enum_options(language, enum_options):
    return tuple(
        {
            'value': option.value,
            'label': _translate_enum_label(language, option.label),
        }
        for option in enum_options
    )


def build_override_choices(registry, language=_LANGUAGE_EN):
    return [
        {
            'field_name': descriptor.name,
            'text': _override_choice_text(language, descriptor),
            'group': _translate_group(language, descriptor.group),
            'label': _translate_field_label(language, descriptor),
            'value_type': descriptor.value_type.value,
            'supports_pulse': descriptor.supports_pulse,
            'enum_options': _translated_enum_options(language, descriptor.enum_options),
        }
        for descriptor in registry
    ]


def build_workflow_choices(workflows_map):
    return [{'name': workflow_name, 'text': workflow_name} for workflow_name in workflows_map.keys()]


def build_trace_mode_choices(language=_LANGUAGE_EN):
    return [
        {'name': _TRACE_MODE_KEY, 'text': _text(language, 'trace_mode_key')},
        {'name': _TRACE_MODE_RAW, 'text': _text(language, 'trace_mode_raw')},
    ]


def format_mask_preview(active_values, registry, language=_LANGUAGE_EN):
    active_field_names = []
    mask = 0
    for descriptor in registry:
        if descriptor.name not in active_values or descriptor.bit is None:
            continue
        mask |= descriptor.bit
        active_field_names.append(descriptor.name)
    summary = ', '.join(active_field_names) if active_field_names else _text(language, 'trace_empty_summary')
    return '{decimal} | {hex_value} | {fields}'.format(
        decimal=_text(language, 'trace_mask_decimal', mask=mask),
        hex_value=_text(language, 'trace_mask_hex', mask=mask),
        fields=_text(language, 'trace_mask_fields', summary=summary),
    )


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


def _format_reason(entry, language=_LANGUAGE_EN):
    details = []
    if entry.transition_reason:
        details.append('{label}: {value}'.format(label=_text(language, 'reason_transition'), value=entry.transition_reason))
    if entry.command_reason:
        details.append('{label}: {value}'.format(label=_text(language, 'reason_command'), value=entry.command_reason))
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

        self._language = _LANGUAGE_EN
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
        self._trace_paused = False
        self._override_active_values = {}
        self._sensor_widgets = {}
        self._sensor_labels = {}
        self._sensor_groups = {}
        self._override_descriptor_map = _build_field_map(self._override_registry)
        self._trace_signal_proxy = _TraceSignalProxy()
        self._trace_signal_proxy.received.connect(self._apply_trace_message)

        self._bind_static_widgets()
        self._build_sensor_inputs()
        self._configure_override_controls()
        self._configure_workflow_controls()
        self._configure_trace_controls()
        self._bind_actions()
        self._apply_language(self._language)
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
            self._set_offline_hint(self._connection_note)

        add_widget = getattr(context, 'add_widget', None)
        if callable(add_widget):
            add_widget(self._widget)

    def _bind_static_widgets(self):
        self.languageLabel = self._widget.findChild(QLabel, 'languageLabel')
        self.languageComboBox = self._widget.findChild(QComboBox, 'languageComboBox')
        self.headerSubtitleLabel = self._widget.findChild(QLabel, 'headerSubtitleLabel')
        self.namespaceValueLineEdit = self._widget.findChild(QLineEdit, 'namespaceValueLineEdit')
        self.namespaceLabel = self._widget.findChild(QLabel, 'namespaceLabel')
        self.overviewTitleLabel = self._widget.findChild(QLabel, 'overviewTitleLabel')
        self.overviewNamespaceTitleLabel = self._widget.findChild(QLabel, 'overviewNamespaceTitleLabel')
        self.currentStateTitleLabel = self._widget.findChild(QLabel, 'currentStateTitleLabel')
        self.overviewNamespaceValueLabel = self._widget.findChild(QLabel, 'overviewNamespaceValueLabel')
        self.currentStateValueLabel = self._widget.findChild(QLabel, 'currentStateValueLabel')
        self.previousStateTitleLabel = self._widget.findChild(QLabel, 'previousStateTitleLabel')
        self.previousStateValueLabel = self._widget.findChild(QLabel, 'previousStateValueLabel')
        self.lastEventTitleLabel = self._widget.findChild(QLabel, 'lastEventTitleLabel')
        self.lastEventValueLabel = self._widget.findChild(QLabel, 'lastEventValueLabel')
        self.outputModeTitleLabel = self._widget.findChild(QLabel, 'outputModeTitleLabel')
        self.outputModeValueLabel = self._widget.findChild(QLabel, 'outputModeValueLabel')
        self.reasonTitleLabel = self._widget.findChild(QLabel, 'reasonTitleLabel')
        self.reasonValueLabel = self._widget.findChild(QLabel, 'reasonValueLabel')
        self.sensorInputsTitleLabel = self._widget.findChild(QLabel, 'sensorInputsTitleLabel')
        self.presetBaseReadyButton = self._widget.findChild(QPushButton, 'presetBaseReadyButton')
        self.presetCommsLossButton = self._widget.findChild(QPushButton, 'presetCommsLossButton')
        self.presetObstacleButton = self._widget.findChild(QPushButton, 'presetObstacleButton')
        self.presetClearButton = self._widget.findChild(QPushButton, 'presetClearButton')
        self.publishSensorInputButton = self._widget.findChild(QPushButton, 'publishSensorInputButton')
        self.overrideComposerTitleLabel = self._widget.findChild(QLabel, 'overrideComposerTitleLabel')
        self.overrideFieldLabel = self._widget.findChild(QLabel, 'overrideFieldLabel')
        self.overrideFieldComboBox = self._widget.findChild(QComboBox, 'overrideFieldComboBox')
        self.overrideActionLabel = self._widget.findChild(QLabel, 'overrideActionLabel')
        self.overrideActionComboBox = self._widget.findChild(QComboBox, 'overrideActionComboBox')
        self.overrideValueLabel = self._widget.findChild(QLabel, 'overrideValueLabel')
        self.overrideValueStackedWidget = self._widget.findChild(QStackedWidget, 'overrideValueStackedWidget')
        self.overrideBoolValueCheckBox = self._widget.findChild(QCheckBox, 'overrideBoolValueCheckBox')
        self.overrideEnumValueComboBox = self._widget.findChild(QComboBox, 'overrideEnumValueComboBox')
        self.overrideFloatValueSpinBox = self._widget.findChild(QDoubleSpinBox, 'overrideFloatValueSpinBox')
        self.overrideActionHintLabel = self._widget.findChild(QLabel, 'overrideActionHintLabel')
        self.overrideMaskPreviewLabel = self._widget.findChild(QLabel, 'overrideMaskPreviewLabel')
        self.sendOverrideButton = self._widget.findChild(QPushButton, 'sendOverrideButton')
        self.workflowTitleLabel = self._widget.findChild(QLabel, 'workflowTitleLabel')
        self.workflowComboBox = self._widget.findChild(QComboBox, 'workflowComboBox')
        self.workflowStatusLabel = self._widget.findChild(QLabel, 'workflowStatusLabel')
        self.runWorkflowButton = self._widget.findChild(QPushButton, 'runWorkflowButton')
        self.traceTitleLabel = self._widget.findChild(QLabel, 'traceTitleLabel')
        self.traceModeLabel = self._widget.findChild(QLabel, 'traceModeLabel')
        self.traceModeComboBox = self._widget.findChild(QComboBox, 'traceModeComboBox')
        self.tracePauseButton = self._widget.findChild(QPushButton, 'tracePauseButton')
        self.traceClearButton = self._widget.findChild(QPushButton, 'traceClearButton')
        self.traceListWidget = self._widget.findChild(QWidget, 'traceListWidget')
        self.sensorGroupsLayout = self._widget.findChild(QWidget, 'sensorScrollAreaContents').layout()
        self.namespaceValueLineEdit.setText(self._namespace)
        self.languageComboBox.blockSignals(True)
        self.languageComboBox.clear()
        for text, value in _LANGUAGE_ITEMS:
            self.languageComboBox.addItem(text, value)
        self.languageComboBox.blockSignals(False)
        self.languageComboBox.currentIndexChanged.connect(self._handle_language_change)

    def _build_sensor_inputs(self):
        grouped_descriptors = {}
        for descriptor in self._sensor_registry:
            grouped_descriptors.setdefault(descriptor.group, []).append(descriptor)

        for group_name, descriptors in grouped_descriptors.items():
            group_box = QGroupBox(group_name)
            group_layout = QGridLayout(group_box)
            group_layout.setHorizontalSpacing(12)
            group_layout.setVerticalSpacing(8)
            self._sensor_groups[group_name] = group_box
            for row, descriptor in enumerate(descriptors):
                label = QLabel(descriptor.label)
                control = self._create_input_widget(descriptor, 'sensorInput__{name}'.format(name=descriptor.name))
                group_layout.addWidget(label, row, 0)
                group_layout.addWidget(control, row, 1)
                self._sensor_widgets[descriptor.name] = control
                self._sensor_labels[descriptor.name] = label
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
        self.overrideFloatValueSpinBox.setSingleStep(0.05)
        self._populate_override_fields()
        self._populate_override_actions()
        self._sync_override_editor()

    def _configure_workflow_controls(self):
        for choice in build_workflow_choices(self._workflows_map):
            self.workflowComboBox.addItem(choice['text'], choice['name'])

    def _configure_trace_controls(self):
        self._populate_trace_modes()
        self.traceListWidget.setAlternatingRowColors(True)

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
        self.traceModeComboBox.currentIndexChanged.connect(self._handle_trace_mode_change)
        self.tracePauseButton.clicked.connect(self._toggle_trace_pause)
        self.traceClearButton.clicked.connect(self._clear_trace_history)

    def _populate_override_fields(self):
        current_field_name = self.overrideFieldComboBox.currentData()
        self.overrideFieldComboBox.blockSignals(True)
        self.overrideFieldComboBox.clear()
        for choice in build_override_choices(self._override_registry, self._language):
            self.overrideFieldComboBox.addItem(choice['text'], choice['field_name'])
        self._set_combo_current_data(self.overrideFieldComboBox, current_field_name)
        self.overrideFieldComboBox.blockSignals(False)

    def _populate_override_actions(self):
        current_action = self.overrideActionComboBox.currentData()
        self.overrideActionComboBox.blockSignals(True)
        self.overrideActionComboBox.clear()
        self.overrideActionComboBox.addItem(_text(self._language, 'override_action_pulse'), _ACTION_PULSE)
        self.overrideActionComboBox.addItem(_text(self._language, 'override_action_latch'), _ACTION_LATCH)
        self.overrideActionComboBox.addItem(_text(self._language, 'override_action_clear'), _ACTION_CLEAR)
        self._set_combo_current_data(self.overrideActionComboBox, current_action)
        self.overrideActionComboBox.blockSignals(False)

    def _populate_trace_modes(self):
        current_mode = self.traceModeComboBox.currentData() or _TRACE_MODE_KEY
        self.traceModeComboBox.blockSignals(True)
        self.traceModeComboBox.clear()
        for choice in build_trace_mode_choices(self._language):
            self.traceModeComboBox.addItem(choice['text'], choice['name'])
        self._set_combo_current_data(self.traceModeComboBox, current_mode)
        self.traceModeComboBox.blockSignals(False)

    def _handle_language_change(self, *_args):
        language = self.languageComboBox.currentData() if self.languageComboBox is not None else _LANGUAGE_EN
        self._apply_language(language)

    def _apply_language(self, language):
        self._language = _language_key(language)
        self._apply_static_texts()
        self._apply_sensor_language()
        self._populate_override_fields()
        self._populate_override_actions()
        self._apply_override_language()
        self._apply_workflow_language()
        self._apply_overview_titles()
        self._apply_trace_language()
        self._apply_workflow_status()

    def _apply_static_texts(self):
        self.languageLabel.setText(_text(self._language, 'language'))
        self.headerSubtitleLabel.setText(_text(self._language, 'header_subtitle'))
        self.namespaceLabel.setText(_text(self._language, 'namespace'))
        self.overviewTitleLabel.setText(_text(self._language, 'overview_title'))
        self.overviewNamespaceTitleLabel.setText(_text(self._language, 'overview_namespace'))
        self.currentStateTitleLabel.setText(_text(self._language, 'current_state'))
        self.previousStateTitleLabel.setText(_text(self._language, 'previous_state'))
        self.lastEventTitleLabel.setText(_text(self._language, 'last_event'))
        self.outputModeTitleLabel.setText(_text(self._language, 'output_mode'))
        self.reasonTitleLabel.setText(_text(self._language, 'reason'))
        self.sensorInputsTitleLabel.setText(_text(self._language, 'sensor_inputs_title'))
        self.presetBaseReadyButton.setText(_text(self._language, 'preset_base_ready'))
        self.presetCommsLossButton.setText(_text(self._language, 'preset_comms_loss'))
        self.presetObstacleButton.setText(_text(self._language, 'preset_obstacle'))
        self.presetClearButton.setText(_text(self._language, 'preset_clear'))
        self.publishSensorInputButton.setText(_text(self._language, 'publish_sensor_input'))
        self.overrideComposerTitleLabel.setText(_text(self._language, 'override_title'))
        self.overrideFieldLabel.setText(_text(self._language, 'override_field'))
        self.overrideActionLabel.setText(_text(self._language, 'override_action'))
        self.overrideValueLabel.setText(_text(self._language, 'override_value'))
        self.overrideBoolValueCheckBox.setText(_text(self._language, 'override_bool'))
        self.sendOverrideButton.setText(_text(self._language, 'override_send'))
        self.workflowTitleLabel.setText(_text(self._language, 'workflow_title'))
        self.runWorkflowButton.setText(_text(self._language, 'workflow_run'))
        self.traceTitleLabel.setText(_text(self._language, 'trace_title'))

    def _apply_sensor_language(self):
        for descriptor in self._sensor_registry:
            group_box = self._sensor_groups.get(descriptor.group)
            if group_box is not None:
                group_box.setTitle(_translate_group(self._language, descriptor.group))
            label = self._sensor_labels.get(descriptor.name)
            if label is not None:
                label.setText(_translate_field_label(self._language, descriptor))
            widget = self._sensor_widgets.get(descriptor.name)
            if widget is not None and descriptor.value_type == FieldValueType.ENUM:
                current_value = widget.currentData()
                widget.blockSignals(True)
                widget.clear()
                for option in descriptor.enum_options:
                    widget.addItem(_translate_enum_label(self._language, option.label), option.value)
                self._set_combo_current_data(widget, current_value)
                widget.blockSignals(False)

    def _apply_override_language(self):
        descriptor = self._current_override_descriptor()
        if descriptor is not None:
            self._configure_override_value_widget(descriptor)
        self._sync_override_action_text()
        self._refresh_override_preview()

    def _apply_workflow_language(self):
        self.workflowComboBox.blockSignals(True)
        current_workflow = self.workflowComboBox.currentData()
        self.workflowComboBox.clear()
        for choice in build_workflow_choices(self._workflows_map):
            self.workflowComboBox.addItem(choice['text'], choice['name'])
        self._set_combo_current_data(self.workflowComboBox, current_workflow)
        self.workflowComboBox.blockSignals(False)

    def _apply_overview_titles(self):
        self.overviewNamespaceTitleLabel.setText(_text(self._language, 'overview_namespace'))
        self.currentStateTitleLabel.setText(_text(self._language, 'current_state'))
        self.previousStateTitleLabel.setText(_text(self._language, 'previous_state'))
        self.lastEventTitleLabel.setText(_text(self._language, 'last_event'))
        self.outputModeTitleLabel.setText(_text(self._language, 'output_mode'))
        self.reasonTitleLabel.setText(_text(self._language, 'reason'))

    def _apply_trace_language(self):
        self.traceTitleLabel.setText(_text(self._language, 'trace_title'))
        self.traceModeLabel.setText(_text(self._language, 'trace_mode_label'))
        self._populate_trace_modes()
        self.tracePauseButton.setText(_text(self._language, 'trace_resume' if self._trace_paused else 'trace_pause'))
        self.traceClearButton.setText(_text(self._language, 'trace_clear'))
        self._refresh_overview(self._trace_model.latest())
        self._refresh_trace_history()

    def _sync_override_action_text(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return
        pulse_index = self.overrideActionComboBox.findData(_ACTION_PULSE)
        pulse_item = self.overrideActionComboBox.model().item(pulse_index) if pulse_index >= 0 else None
        if pulse_item is not None:
            pulse_item.setEnabled(descriptor.supports_pulse)
        if not descriptor.supports_pulse and self._current_override_action() == _ACTION_PULSE:
            latch_index = self.overrideActionComboBox.findData(_ACTION_LATCH)
            self.overrideActionComboBox.setCurrentIndex(latch_index)
        if descriptor.supports_pulse:
            self.overrideActionHintLabel.setText(_text(self._language, 'override_hint_pulse'))
        else:
            self.overrideActionHintLabel.setText(_text(self._language, 'override_hint_no_pulse'))

    def _apply_workflow_status(self):
        if getattr(self, '_workflow_status_kind', None) == 'queued':
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_queued', name=self._workflow_status_name))
        elif getattr(self, '_workflow_status_kind', None) == 'failed':
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_failed', message=self._workflow_status_message))
        elif getattr(self, '_workflow_status_kind', None) == 'succeeded':
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_succeeded', name=self._workflow_status_name))
        elif getattr(self, '_workflow_status_kind', None) == 'sensor_failed':
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_sensor_failed', message=self._workflow_status_message))
        elif getattr(self, '_workflow_status_kind', None) == 'offline':
            self.overrideActionHintLabel.setText(_text(self._language, 'offline_hint', message=self._workflow_status_message))
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_idle'))
        else:
            self.workflowStatusLabel.setText(_text(self._language, 'workflow_idle'))

    def _set_offline_hint(self, message):
        self._workflow_status_kind = 'offline'
        self._workflow_status_message = message
        self._workflow_status_name = ''
        self.overrideActionHintLabel.setText(_text(self._language, 'offline_hint', message=message))

    def _set_workflow_status(self, kind, name='', message=''):
        self._workflow_status_kind = kind
        self._workflow_status_name = name
        self._workflow_status_message = message
        self._apply_workflow_status()

    def _sync_override_editor(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return

        self._configure_override_value_widget(descriptor)
        self._sync_override_action_text()
        self._refresh_override_preview()

    def _apply_sensor_preset(self, payload):
        for descriptor in self._sensor_registry:
            self._set_widget_value(self._sensor_widgets[descriptor.name], descriptor, payload.get(descriptor.name, descriptor.default_value))

    def _publish_sensor_input(self):
        try:
            self._topic_facade.publish_sensor_input(self._collect_sensor_values())
        except Exception as exc:
            self._set_workflow_status('sensor_failed', message=str(exc))

    def _configure_override_value_widget(self, descriptor):
        current_value = self._override_active_values.get(descriptor.name, descriptor.default_value)
        if descriptor.value_type == FieldValueType.BOOL:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideBoolValueCheckBox.parentWidget())
            bool_value = current_value
            if descriptor.name not in self._override_active_values:
                if descriptor.supports_pulse and self._current_override_action() == _ACTION_PULSE:
                    bool_value = True
            self.overrideBoolValueCheckBox.setChecked(bool(bool_value))
        elif descriptor.value_type == FieldValueType.ENUM:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideEnumValueComboBox.parentWidget())
            self.overrideEnumValueComboBox.blockSignals(True)
            self.overrideEnumValueComboBox.clear()
            for option in descriptor.enum_options:
                self.overrideEnumValueComboBox.addItem(_translate_enum_label(self._language, option.label), option.value)
            self._set_combo_to_value(self.overrideEnumValueComboBox, current_value)
            self.overrideEnumValueComboBox.blockSignals(False)
        else:
            self.overrideValueStackedWidget.setCurrentWidget(self.overrideFloatValueSpinBox.parentWidget())
            self.overrideFloatValueSpinBox.setValue(float(current_value or 0.0))

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
        self.overrideMaskPreviewLabel.setText(format_mask_preview(projected, self._override_registry, self._language))

    def _send_override(self):
        descriptor = self._current_override_descriptor()
        if descriptor is None:
            return

        action = self._current_override_action()
        value = self._read_widget_value(self._current_override_value_widget(), descriptor)
        try:
            if action == _ACTION_PULSE:
                self._topic_facade.pulse_override(descriptor.name, value)
            elif action == _ACTION_LATCH:
                self._topic_facade.latch_override(descriptor.name, value)
            else:
                self._topic_facade.clear_override(descriptor.name)
        except Exception as exc:
            self.overrideActionHintLabel.setText(_text(self._language, 'override_publish_failed', message=str(exc)))
            self.overrideMaskPreviewLabel.setText(
                format_mask_preview(self._override_active_values, self._override_registry, self._language)
            )
            return

        if action == _ACTION_LATCH:
            self._override_active_values[descriptor.name] = value
        elif action == _ACTION_CLEAR:
            self._override_active_values.pop(descriptor.name, None)
        self._refresh_override_preview()

    def _run_workflow(self):
        workflow_name = self.workflowComboBox.currentData()
        if workflow_name:
            self._set_workflow_status('queued', name=workflow_name)
            try:
                self._workflow_runner.run(workflow_name)
            except Exception as exc:
                self._set_workflow_status('failed', name=workflow_name, message=str(exc))
            else:
                self._set_workflow_status('succeeded', name=workflow_name)

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
    def _set_combo_current_data(combo_box, value):
        if value is None:
            return
        index = combo_box.findData(value)
        if index >= 0:
            combo_box.setCurrentIndex(index)

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
        self.reasonValueLabel.setText(_format_reason(entry, self._language))

    def _handle_trace_message(self, message):
        self._trace_signal_proxy.received.emit(message)

    def _apply_trace_message(self, message):
        entry = self._trace_model.update(message)
        self._refresh_overview(entry)
        if self._trace_paused:
            return
        self._apply_trace_update()

    def _refresh_trace_history(self):
        self.traceListWidget.clear()
        for history_entry in self._visible_trace_history():
            self.traceListWidget.addItem(self._build_trace_item(history_entry))

    def _apply_trace_update(self):
        delta = self._trace_model.last_update_delta()
        if delta is None:
            return

        if self._current_trace_mode() == _TRACE_MODE_RAW:
            self.traceListWidget.insertItem(0, self._build_trace_item(delta.raw_entry))
        elif delta.key_appended or self.traceListWidget.count() == 0:
            self.traceListWidget.insertItem(0, self._build_trace_item(delta.key_entry))
        else:
            self._replace_trace_item(0, delta.key_entry)

        visible_count = len(self._visible_trace_history())
        while self.traceListWidget.count() > visible_count:
            self.traceListWidget.takeItem(self.traceListWidget.count() - 1)

    def _build_trace_item(self, history_entry):
        item = QListWidgetItem(self._format_trace_row(history_entry))
        self._apply_trace_item_background(item, history_entry.entry.current_state)
        return item

    def _replace_trace_item(self, row, history_entry):
        item = self.traceListWidget.item(row)
        if item is None:
            self.traceListWidget.insertItem(row, self._build_trace_item(history_entry))
            return
        item.setText(self._format_trace_row(history_entry))
        self._apply_trace_item_background(item, history_entry.entry.current_state)

    def _format_trace_row(self, history_entry):
        entry = history_entry.entry
        segments = [
            '{current_label}: {current}'.format(
                current_label=_text(self._language, 'trace_current'),
                current=entry.current_state or '-',
            ),
            '{event_label}: {event}'.format(
                event_label=_text(self._language, 'trace_event'),
                event=entry.last_event or '-',
            ),
            '{previous_label}: {previous}'.format(
                previous_label=_text(self._language, 'trace_previous'),
                previous=entry.previous_state or '-',
            ),
            '{mode_label}: {mode}'.format(
                mode_label=_text(self._language, 'trace_mode'),
                mode=entry.output_mode or '-',
            ),
            _format_reason(entry, self._language),
        ]
        return ' | '.join(segments)

    @staticmethod
    def _apply_trace_item_background(item, current_state):
        highlight = _TRACE_HIGHLIGHTS.get(current_state)
        if highlight is None:
            item.setData(Qt.BackgroundRole, None)
            return
        item.setBackground(highlight)

    def _visible_trace_history(self):
        if self._current_trace_mode() == _TRACE_MODE_RAW:
            return self._trace_model.raw_history()
        return self._trace_model.key_history()

    def _handle_trace_mode_change(self, *_args):
        self._refresh_trace_history()

    def _toggle_trace_pause(self):
        self._trace_paused = not self._trace_paused
        self.tracePauseButton.setText(_text(self._language, 'trace_resume' if self._trace_paused else 'trace_pause'))
        if not self._trace_paused:
            self._refresh_trace_history()

    def _clear_trace_history(self):
        self._trace_model.clear_history()
        self._refresh_trace_history()

    def _current_trace_mode(self):
        return self.traceModeComboBox.currentData() or _TRACE_MODE_KEY

    def _wait_for_state(self, expected_state, timeout_sec):
        deadline = time.time() + float(timeout_sec or 0.0)
        while time.time() <= deadline:
            latest = self._trace_model.latest()
            if latest and latest.current_state == expected_state:
                return True
            QApplication.processEvents()
            time.sleep(0.02)
        return False
