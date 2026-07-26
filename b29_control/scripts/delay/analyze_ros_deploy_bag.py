#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rosbag
import yaml


ACTIVE_TOPICS = (
    "/joint_states",
    "/gp11/rl/observation",
    "/gp11/rl/action_raw",
    "/gp11/rl/target_point_local",
)

TERM_SLICES = {
    "q_norm": slice(0, 4),
    "dq_norm": slice(4, 8),
    "last_action": slice(8, 12),
    "lower_margin": slice(12, 16),
    "upper_margin": slice(16, 20),
    "goal_pos": slice(20, 23),
    "goal_x_axis": slice(23, 26),
    "active_side": slice(26, 28),
}


def _as_float_array(values: Sequence[float], size: int, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (size,):
        raise ValueError(f"{name} must have shape ({size},), got {arr.shape}")
    return arr


def _stats(arrays: Sequence[np.ndarray]) -> Dict[str, Any]:
    if not arrays:
        return {"count": 0}
    data = np.asarray(arrays, dtype=np.float64)
    flat_abs = np.abs(data.reshape(-1))
    return {
        "count": int(data.shape[0]),
        "shape": list(data.shape[1:]),
        "min": np.min(data, axis=0).tolist(),
        "max": np.max(data, axis=0).tolist(),
        "mean": np.mean(data, axis=0).tolist(),
        "std": np.std(data, axis=0).tolist(),
        "abs_p95": float(np.percentile(flat_abs, 95)),
        "abs_max": float(np.max(flat_abs)),
    }


def _error_stats(errors: Sequence[np.ndarray]) -> Dict[str, Any]:
    if not errors:
        return {"count": 0}
    data = np.abs(np.asarray(errors, dtype=np.float64))
    return {
        "count": int(data.shape[0]),
        "mean_abs": np.mean(data, axis=0).tolist(),
        "max_abs": np.max(data, axis=0).tolist(),
        "overall_mean_abs": float(np.mean(data)),
        "overall_max_abs": float(np.max(data)),
        "overall_p95_abs": float(np.percentile(data.reshape(-1), 95)),
    }


def _rate_stats(times: Sequence[float]) -> Dict[str, Any]:
    if len(times) < 2:
        return {"count": len(times)}
    dt = np.diff(np.asarray(times, dtype=np.float64))
    return {
        "count": len(times),
        "duration": float(times[-1] - times[0]),
        "hz_mean": float(1.0 / np.mean(dt)) if np.mean(dt) > 0 else None,
        "dt_mean": float(np.mean(dt)),
        "dt_std": float(np.std(dt)),
        "dt_min": float(np.min(dt)),
        "dt_max": float(np.max(dt)),
    }


def _load_runtime(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    runtime = raw["runtime"]
    deploy = raw["deploy"]
    return {
        "anchor_side": str(runtime["anchor_side"]).strip().lower(),
        "active_dof_names": list(runtime["active_dof_names"]),
        "hard_lower": _as_float_array(runtime["hard_lower"], 4, "hard_lower"),
        "hard_upper": _as_float_array(runtime["hard_upper"], 4, "hard_upper"),
        "velocity_limits": _as_float_array(runtime["velocity_limits"], 4, "velocity_limits"),
        "clip_observations": float(runtime["clip_observations"]),
        "dq_alpha": float(deploy.get("obs_filter", {}).get("dq_alpha", 1.0)),
    }


def _side_one_hot(anchor_side: str) -> np.ndarray:
    if anchor_side == "right":
        return np.asarray([1.0, 0.0], dtype=np.float64)
    if anchor_side == "left":
        return np.asarray([0.0, 1.0], dtype=np.float64)
    raise ValueError(f"unsupported anchor_side: {anchor_side}")


def _build_obs(
    q: np.ndarray,
    dq_filtered: np.ndarray,
    last_action: np.ndarray,
    target_pos: np.ndarray,
    runtime: Dict[str, Any],
) -> np.ndarray:
    hard_lower = runtime["hard_lower"]
    hard_upper = runtime["hard_upper"]
    q_mid = 0.5 * (hard_lower + hard_upper)
    q_half = np.maximum(0.5 * (hard_upper - hard_lower), 1e-6)
    q_range = np.maximum(hard_upper - hard_lower, 1e-6)
    vel_div = np.maximum(runtime["velocity_limits"], 1.0)
    obs = np.concatenate(
        [
            (q - q_mid) / q_half,
            dq_filtered / vel_div,
            last_action,
            (q - hard_lower) / q_range,
            (hard_upper - q) / q_range,
            target_pos,
            np.asarray([1.0, 0.0, 0.0], dtype=np.float64),
            _side_one_hot(runtime["anchor_side"]),
        ]
    )
    return np.clip(obs, -runtime["clip_observations"], runtime["clip_observations"])


def _reorder(names: Sequence[str], values: Sequence[float], active_names: Sequence[str]) -> np.ndarray:
    index = {name: i for i, name in enumerate(names)}
    missing = [name for name in active_names if name not in index]
    if missing:
        raise KeyError(f"missing joints: {missing}")
    arr = np.asarray(values, dtype=np.float64)
    return arr[[index[name] for name in active_names]]


def _nearest_delta(query_t: float, times: Sequence[float]) -> Optional[float]:
    if not times:
        return None
    idx = int(np.searchsorted(times, query_t))
    candidates = []
    if idx < len(times):
        candidates.append(abs(times[idx] - query_t))
    if idx > 0:
        candidates.append(abs(times[idx - 1] - query_t))
    return float(min(candidates)) if candidates else None


def analyze_bag(bag_path: Path, config_path: Path, dq_alpha_override: Optional[float] = None) -> Dict[str, Any]:
    runtime = _load_runtime(config_path)
    if dq_alpha_override is not None:
        runtime["dq_alpha"] = float(dq_alpha_override)
    active_names = runtime["active_dof_names"]
    dq_alpha = runtime["dq_alpha"]

    times: Dict[str, List[float]] = {topic: [] for topic in ACTIVE_TOPICS}
    q_values: List[np.ndarray] = []
    dq_raw_values: List[np.ndarray] = []
    dq_filtered_values: List[np.ndarray] = []
    obs_values: List[np.ndarray] = []
    action_values: List[np.ndarray] = []
    target_values: List[np.ndarray] = []
    target_norm_values: List[np.ndarray] = []
    policy_target_q_values: List[np.ndarray] = []
    policy_target_minus_q_errors: List[np.ndarray] = []
    obs_rebuild_errors: List[np.ndarray] = []
    obs_term_errors: Dict[str, List[np.ndarray]] = {name: [] for name in TERM_SLICES}
    last_action_prev_errors: List[np.ndarray] = []
    last_action_same_tick_errors: List[np.ndarray] = []
    target_to_obs_time_delta: List[float] = []
    joint_to_obs_time_delta: List[float] = []
    missing_joint_messages = 0
    observed_joint_name_orders: Dict[Tuple[str, ...], int] = {}

    latest_q: Optional[np.ndarray] = None
    latest_dq_filtered = np.zeros(4, dtype=np.float64)
    latest_target: Optional[np.ndarray] = None
    prev_action = np.zeros(4, dtype=np.float64)
    pending_obs_last_action: Optional[np.ndarray] = None
    last_joint_time: Optional[float] = None
    last_target_time: Optional[float] = None

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, stamp in bag.read_messages(topics=list(ACTIVE_TOPICS)):
            t = float(stamp.to_sec())
            times[topic].append(t)
            if topic == "/joint_states":
                names = tuple(msg.name)
                observed_joint_name_orders[names] = observed_joint_name_orders.get(names, 0) + 1
                try:
                    q = _reorder(msg.name, msg.position, active_names)
                    if len(msg.velocity) >= len(msg.name):
                        dq_raw = _reorder(msg.name, msg.velocity, active_names)
                    else:
                        dq_raw = np.zeros(4, dtype=np.float64)
                    latest_dq_filtered = dq_alpha * dq_raw + (1.0 - dq_alpha) * latest_dq_filtered
                    latest_q = q
                    last_joint_time = t
                    q_values.append(q)
                    dq_raw_values.append(dq_raw)
                    dq_filtered_values.append(latest_dq_filtered.copy())
                except KeyError:
                    missing_joint_messages += 1
            elif topic == "/gp11/rl/target_point_local":
                if len(msg.data) >= 3:
                    latest_target = np.asarray(msg.data[:3], dtype=np.float64)
                    last_target_time = t
                    target_values.append(latest_target)
                    target_norm_values.append(np.asarray([np.linalg.norm(latest_target)], dtype=np.float64))
            elif topic == "/gp11/rl/observation":
                obs = np.asarray(msg.data, dtype=np.float64)
                obs_values.append(obs)
                if obs.shape == (28,):
                    pending_obs_last_action = obs[TERM_SLICES["last_action"]].copy()
                    last_action_prev_errors.append(pending_obs_last_action - prev_action)
                    if latest_q is not None and latest_target is not None:
                        expected = _build_obs(latest_q, latest_dq_filtered, prev_action, latest_target, runtime)
                        err = obs - expected
                        obs_rebuild_errors.append(err)
                        for name, slc in TERM_SLICES.items():
                            obs_term_errors[name].append(err[slc])
                        if last_joint_time is not None:
                            joint_to_obs_time_delta.append(t - last_joint_time)
                        if last_target_time is not None:
                            target_to_obs_time_delta.append(t - last_target_time)
            elif topic == "/gp11/rl/action_raw":
                action = np.asarray(msg.data, dtype=np.float64)
                if action.shape == (4,):
                    action_values.append(action)
                    if latest_q is not None:
                        clipped_action = np.clip(action, -1.0, 1.0)
                        policy_target_q = (
                            runtime["hard_lower"]
                            + 0.5 * (clipped_action + 1.0) * (runtime["hard_upper"] - runtime["hard_lower"])
                        )
                        policy_target_q_values.append(policy_target_q)
                        policy_target_minus_q_errors.append(policy_target_q - latest_q)
                    if pending_obs_last_action is not None:
                        last_action_same_tick_errors.append(pending_obs_last_action - action)
                        pending_obs_last_action = None
                    prev_action = np.clip(action, -1.0, 1.0)

    obs_dims = sorted({int(v.shape[0]) for v in obs_values})
    action_dims = sorted({int(v.shape[0]) for v in action_values})
    target_dims = sorted({int(v.shape[0]) for v in target_values})
    joint_orders = [
        {"count": count, "names": list(names)}
        for names, count in sorted(observed_joint_name_orders.items(), key=lambda item: -item[1])[:5]
    ]

    return {
        "bag": str(bag_path),
        "config": str(config_path),
        "runtime": {
            "anchor_side": runtime["anchor_side"],
            "active_dof_names": runtime["active_dof_names"],
            "dq_alpha": runtime["dq_alpha"],
        },
        "topic_rates": {topic: _rate_stats(topic_times) for topic, topic_times in times.items()},
        "message_shapes": {
            "observation_dims": obs_dims,
            "action_raw_dims": action_dims,
            "target_point_dims": target_dims,
        },
        "joint_name_orders_top": joint_orders,
        "missing_joint_messages": missing_joint_messages,
        "stats": {
            "q": _stats(q_values),
            "dq_raw": _stats(dq_raw_values),
            "dq_filtered": _stats(dq_filtered_values),
            "observation": _stats(obs_values),
            "action_raw": _stats(action_values),
            "target_point_local": _stats(target_values),
            "target_point_norm": _stats(target_norm_values),
            "policy_target_q_from_action_raw": _stats(policy_target_q_values),
        },
        "checks": {
            "q_limit_violation": {
                "count": len(q_values),
                "above_upper_count": (
                    np.sum(np.asarray(q_values) > runtime["hard_upper"], axis=0).astype(int).tolist()
                    if q_values
                    else []
                ),
                "below_lower_count": (
                    np.sum(np.asarray(q_values) < runtime["hard_lower"], axis=0).astype(int).tolist()
                    if q_values
                    else []
                ),
                "max_upper_excess": (
                    np.maximum(np.asarray(q_values) - runtime["hard_upper"], 0.0).max(axis=0).tolist()
                    if q_values
                    else []
                ),
                "max_lower_excess": (
                    np.maximum(runtime["hard_lower"] - np.asarray(q_values), 0.0).max(axis=0).tolist()
                    if q_values
                    else []
                ),
            },
            "policy_target_minus_q": _error_stats(policy_target_minus_q_errors),
            "obs_rebuild_error": _error_stats(obs_rebuild_errors),
            "obs_term_errors": {name: _error_stats(errors) for name, errors in obs_term_errors.items()},
            "last_action_vs_previous_action": _error_stats(last_action_prev_errors),
            "last_action_vs_same_tick_action": _error_stats(last_action_same_tick_errors),
            "joint_to_obs_time_delta": _stats([np.asarray([x]) for x in joint_to_obs_time_delta]),
            "target_to_obs_time_delta": _stats([np.asarray([x]) for x in target_to_obs_time_delta]),
        },
    }


def _diff_stats(a: Dict[str, Any], b: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for group in ("q", "dq_raw", "dq_filtered", "observation", "action_raw", "target_point_local"):
        a_stats = a["stats"][group]
        b_stats = b["stats"][group]
        if not a_stats.get("count") or not b_stats.get("count"):
            continue
        out[group] = {
            "mean_delta_b_minus_a": (
                np.asarray(b_stats["mean"], dtype=np.float64) - np.asarray(a_stats["mean"], dtype=np.float64)
            ).tolist(),
            "std_ratio_b_over_a": (
                np.asarray(b_stats["std"], dtype=np.float64)
                / np.maximum(np.asarray(a_stats["std"], dtype=np.float64), 1e-12)
            ).tolist(),
            "abs_max": {"a": a_stats["abs_max"], "b": b_stats["abs_max"]},
            "abs_p95": {"a": a_stats["abs_p95"], "b": b_stats["abs_p95"]},
        }
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--compare", type=Path, default=None, help="optional second bag, reported as compare_bag")
    parser.add_argument("--dq-alpha", type=float, default=None, help="override deploy.obs_filter.dq_alpha")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    report = {"base": analyze_bag(args.bag, args.config, dq_alpha_override=args.dq_alpha)}
    if args.compare is not None:
        report["compare"] = analyze_bag(args.compare, args.config, dq_alpha_override=args.dq_alpha)
        report["diff_compare_minus_base"] = _diff_stats(report["base"], report["compare"])

    text = json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
