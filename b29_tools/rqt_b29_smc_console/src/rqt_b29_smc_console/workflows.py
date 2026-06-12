from typing import Iterable, Mapping

try:
    from b29_smc_auto_controller.msg import AutoSensorInput
except ImportError:  # pragma: no cover - allows local test/import without ROS runtime
    class AutoSensorInput:  # type: ignore[no-redef]
        OBSTACLE_UNKNOWN = 0

from .models import WorkflowStep

BASE_READY_SENSOR_PAYLOAD = {
    'lower_alive': True,
    'imu_ready': True,
    'grip_confirmed': True,
    'joint_fault': False,
    'grip_fault': False,
    'obstacle_detected': False,
    'obstacle_type': AutoSensorInput.OBSTACLE_UNKNOWN,
    'classification_stable': False,
    'range_to_obstacle': 0.0,
    'at_crossing_position': False,
    'post_check_passed': False,
    'post_check_failed': False,
}


def build_default_workflows():
    return {
        'CommsLoss -> Idle': (
            WorkflowStep(
                action='sensor',
                description='simulate comms loss',
                payload={
                    'lower_alive': False,
                    'imu_ready': False,
                    'grip_confirmed': False,
                    'joint_fault': False,
                    'grip_fault': False,
                    'obstacle_detected': False,
                    'obstacle_type': AutoSensorInput.OBSTACLE_UNKNOWN,
                    'classification_stable': False,
                    'range_to_obstacle': 0.0,
                    'at_crossing_position': False,
                    'post_check_passed': False,
                    'post_check_failed': False,
                },
                expected_state='CommsLoss',
                timeout_sec=1.0,
            ),
            WorkflowStep(
                action='sensor',
                description='restore base sensor readiness',
                payload=dict(BASE_READY_SENSOR_PAYLOAD),
                expected_state='Idle',
                timeout_sec=1.0,
            ),
        ),
        'Idle -> AutoInit -> Traversing': (
            WorkflowStep(
                action='sensor',
                description='restore base sensor readiness',
                payload=dict(BASE_READY_SENSOR_PAYLOAD),
                expected_state='Idle',
                timeout_sec=1.0,
            ),
            WorkflowStep(
                action='latch',
                description='hold posture ready',
                payload={'field': 'posture_ready', 'value': True},
                expected_state='Idle',
                timeout_sec=1.0,
            ),
            WorkflowStep(
                action='pulse',
                description='request auto start',
                payload={'field': 'auto_start_requested', 'value': True},
                expected_state='AutoInit',
                timeout_sec=1.0,
            ),
            WorkflowStep(
                action='sensor',
                description='confirm traversing after auto init',
                payload=dict(BASE_READY_SENSOR_PAYLOAD),
                expected_state='Traversing',
                timeout_sec=2.0,
            ),
            WorkflowStep(
                action='clear',
                description='release posture ready latch',
                payload={'field': 'posture_ready'},
            ),
        ),
        'EmergencyStop -> SafeStop': (
            WorkflowStep(
                action='pulse',
                description='trigger emergency stop',
                payload={'field': 'emergency_stop', 'value': True},
                expected_state='SafeStop',
                timeout_sec=1.0,
            ),
        ),
        'SafeStop -> Idle': (
            WorkflowStep(
                action='pulse',
                description='request manual reset',
                payload={'field': 'manual_reset_requested', 'value': True},
                expected_state='Idle',
                timeout_sec=1.0,
            ),
        ),
    }


class WorkflowRunner:
    def __init__(self, topic_facade, wait_for_state):
        self._topic_facade = topic_facade
        self._wait_for_state = wait_for_state

    def run(self, workflow_or_steps):
        if isinstance(workflow_or_steps, str):
            workflows = build_default_workflows()
            if workflow_or_steps not in workflows:
                raise ValueError(f'Unknown workflow: {workflow_or_steps}')
            steps = workflows[workflow_or_steps]
        else:
            steps = workflow_or_steps

        for step in steps:
            self._execute_step(step)

    def _execute_step(self, step: WorkflowStep):
        if step.action == 'sensor':
            self._topic_facade.publish_sensor_input(dict(step.payload))
        elif step.action == 'latch':
            field_name, value = self._resolve_field_payload(step.payload)
            self._topic_facade.latch_override(field_name, value)
        elif step.action == 'pulse':
            field_name, value = self._resolve_field_payload(step.payload)
            self._topic_facade.pulse_override(field_name, value)
        elif step.action == 'clear':
            field_name, _value = self._resolve_field_payload(step.payload, default_value=None)
            self._topic_facade.clear_override(field_name)
        elif step.action == 'service':
            self._call_service(step.payload)
        else:
            raise ValueError(f'Unsupported workflow action: {step.action}')

        if step.expected_state:
            self._wait_for_expected_state(step)

    def _wait_for_expected_state(self, step: WorkflowStep):
        try:
            result = self._wait_for_state(step.expected_state, step.timeout_sec)
        except Exception as exc:
            raise RuntimeError(
                f'Failed waiting for state {step.expected_state} during step {step.description}'
            ) from exc

        if result is not True:
            raise RuntimeError(f'Failed waiting for state {step.expected_state} during step {step.description}')

    @staticmethod
    def _resolve_field_payload(payload, default_value=True):
        if not isinstance(payload, Mapping):
            raise ValueError('Workflow step payload must be a mapping')
        if 'field' in payload:
            return payload['field'], payload.get('value', default_value)
        if 'field_name' in payload:
            return payload['field_name'], payload.get('value', default_value)
        raise ValueError('Workflow step payload must include field or field_name')

    def _call_service(self, payload):
        if not isinstance(payload, Mapping) or 'name' not in payload:
            raise ValueError('Service workflow payload must include name')
        services = {
            'planner_release': self._topic_facade.call_planner_release,
            'software_emergency_stop': self._topic_facade.call_software_emergency_stop,
            'manual_reset': self._topic_facade.call_manual_reset,
        }
        try:
            callback = services[payload['name']]
        except KeyError as exc:
            raise ValueError(f'Unsupported workflow service: {payload["name"]}') from exc
        response = callback()
        if hasattr(response, 'success') and not response.success:
            raise RuntimeError(getattr(response, 'message', 'service rejected request'))
