"""fit_actuator_id v2 自检。直接运行，不依赖 pytest。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "fit_actuator_id", Path(__file__).resolve().parent / "fit_actuator_id.py")
fa = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fa)


def _step(t, edge, pre, post):
    return np.where(t >= edge, post, pre)


def _group(t, cmd, q, meta, name="C/test/low", kind="sweep"):
    n = len(t)
    return {"name": name, "joint": "test_joint", "kind": kind, "j": 0, "t": t,
            "cmd": np.column_stack([cmd] + [np.zeros(n)] * 3),
            "q": np.column_stack([q] + [np.zeros(n)] * 3),
            "dq": np.zeros((n, 4)), "meta": meta}


def test_step_fit_recovers_first_order_params():
    t = np.arange(0.0, 2.0, 0.01)
    cmd = _step(t, 0.3, 0.0, 0.03)
    q = fa.simulate(t, cmd, 0.06, 6.0, 0.05, 0.0)
    params, model = fa.fit_step_event(t, cmd, q)
    assert abs(params["td"] - 0.06) < 0.012, params
    assert abs(params["tau_m"] - 0.05) < 0.012, params
    assert params["nrmse"] < 0.02 and fa.nrmse(q, model) < 0.02


def test_first_order_absolute_bias_and_gain():
    t = np.arange(0.0, 2.0, 0.01)
    cmd = _step(t, 0.3, 1.25, 1.29)
    q = fa.simulate(t, cmd, 0.06, 10.0, 0.05, -0.42, K=1.35)
    params, _ = fa.fit_step_event(t, cmd, q)
    assert abs(params["cmd_pre"] - 1.25) < 1e-10 and abs(params["q_pre"] + .42) < 1e-10
    assert abs(params["K"] - 1.35) < 0.04 and abs(params["dc_gain"] - 1.35) < 0.04, params


def test_step_fit_recovers_vmax_when_rate_saturated():
    t = np.arange(0.0, 2.0, 0.01)
    cmd = _step(t, 0.3, 0.0, 0.60)
    q = fa.simulate(t, cmd, 0.06, 3.0, 0.02, 0.0)
    params, _ = fa.fit_step_event(t, cmd, q)
    assert abs(params["vmax"] - 3.0) / 3.0 < 0.10 and params["rate_sat_frac"] > 0.5, params


def test_vmax_unidentifiable_when_inertia_dominates():
    t = np.arange(0.0, 2.0, 0.005)
    cmd = _step(t, 0.3, 0.0, 0.03)
    q = fa.simulate(t, cmd, 0.06, 60.0, 0.05, 0.0)
    params, _ = fa.fit_step_event(t, cmd, q)
    assert params["rate_sat_frac"] < 0.6 and not params["rate_identifiable"], params


def test_second_order_recovers_gain_wn_zeta():
    t = np.arange(0.0, 3.0, 0.002)
    cmd = _step(t, 0.5, 1.0, 1.03)
    q = fa.simulate(t, cmd, 0.04, 1e6, None, -0.25, wn=12.0, zeta=0.35, K=1.4)
    params, _ = fa.fit_step_event(t, cmd, q, second_order=True)
    assert abs(params["wn"] - 12.0) / 12.0 < 0.12 and abs(params["zeta"] - .35) < .06, params
    assert abs(params["K"] - 1.4) < .06 and params["nrmse"] < .03, params


def test_second_order_stable_at_high_wn():
    t = np.arange(0.0, 1.0, 0.02)
    q = fa.simulate(t, _step(t, .1, 0.0, .03), .02, 1e6, None, 0.0, wn=180.0, zeta=.08)
    assert np.isfinite(q).all() and np.max(np.abs(q)) < 1.0


def test_optimizer_and_bound_diagnostics():
    t = np.arange(0.0, 1.2, .01)
    q = fa.simulate(t, _step(t, .2, 0.0, 1.0), .29, .2, .6, 0.0, K=.1)
    params, _ = fa.fit_step_event(t, _step(t, .2, 0.0, 1.0), q)
    assert set(params["optimizer"]) == {"success", "status", "message", "cost", "optimality", "nfev", "active_mask"}
    assert all(key in params for key in params["optimizer"])
    assert params["optimizer"]["nfev"] > 0
    assert params["any_bound_hit"] and not params["fit_valid"], params


def test_td_geom_tracks_true_delay():
    t = np.arange(0.0, 1.5, .002)
    cmd = _step(t, .3, 0.0, .03)
    q = fa.simulate(t, cmd, .06, 6.0, .05, 0.0)
    params, _ = fa.fit_step_event(t, cmd, q)
    assert abs(params["td_geom"] - .06) < .012, params


def test_falling_edge_recovers_params():
    t = np.arange(0.0, 2.0, .01)
    cmd = _step(t, .3, .03, 0.0)
    q = fa.simulate(t, cmd, .06, 6.0, .05, .03)
    params, _ = fa.fit_step_event(t, cmd, q)
    assert abs(params["td"] - .06) < .012 and abs(params["tau_m"] - .05) < .012, params


def test_find_step_edges_counts_rising_and_falling():
    t = np.arange(0.0, 6.0, .02); cmd = np.zeros_like(t)
    for k in range(2): cmd[(t >= 1 + 2 * k) & (t < 2 + 2 * k)] = .03
    assert len(fa.find_step_edges(t, cmd, .03)) == 4


def test_process_step_group_splits_by_direction():
    t = np.arange(0.0, 6.0, .02); cmd = np.zeros_like(t)
    for k in range(2): cmd[(t >= 1 + 2 * k) & (t < 2 + 2 * k)] = .03
    q = fa.simulate(t, cmd, .06, 6.0, .05, 0.0)
    result = fa.process_step_group(_group(t, cmd, q, {"amplitude": .03}, "B/test/+0.03", "step"), False, ".", False)
    assert set(result["agg_by_dir"]) == {"+", "-"}
    assert sum(e["dir"] == "+" for e in result["events"]) == 2
    assert sum(e["dir"] == "-" for e in result["events"]) == 2


def test_sine_fit_amplitude_phase_and_trend():
    t = np.arange(0.0, 4.0, .01)
    amp, phase = fa.sine_fit(t, .7 * np.sin(2*np.pi*1.5*t+.4) + .1 + .01*t, 1.5)
    assert abs(amp-.7) < 1e-6 and abs(phase-.4) < 1e-6


def test_sweep_points_legacy_compatibility():
    t = np.arange(0.0, 20.0, .002); f = 1.5
    cmd = .03 * np.sin(2*np.pi*f*t); q = fa.simulate(t, cmd, .06, 1e6, .05, 0.0)
    points = fa.sweep_points(t[1000:]-t[1000], cmd[1000:], q[1000:], [f])
    mag, lag = fa.model_bode(f, td=.06, tau=.05)
    assert len(points) == 1 and abs(points[0]["amp_ratio"] - mag) < .03 and abs(points[0]["phase_lag_deg"] - lag) < 5


def test_sweep_points_skips_absent_frequency():
    t = np.arange(0.0, 5.0, .01)
    cmd = .03 * np.sin(2*np.pi*.5*t)
    points = fa.sweep_points(t, cmd, cmd.copy(), [.5, 7.0])
    assert [point["freq_hz"] for point in points] == [.5]


def _protocol_sweep(freqs, response_fn, dt=.002):
    t_all, c_all, q_all, cursor = [], [], [], 0.0
    for f in freqs:
        hold = np.arange(cursor, cursor + 1.0, dt); t_all.append(hold); c_all.append(np.zeros_like(hold)); q_all.append(np.zeros_like(hold)); cursor += 1.0
        cycles = 6 if f < 1 else 10; sine = np.arange(cursor, cursor + cycles/f, dt)
        local = sine - cursor; c = .03*np.sin(2*np.pi*f*local); q = response_fn(f, local, c)
        t_all.append(sine); c_all.append(c); q_all.append(q); cursor += cycles/f
    return np.concatenate(t_all), np.concatenate(c_all), np.concatenate(q_all)


def test_segmented_low_sweep_has_no_cross_frequency_leakage():
    freqs = [.5, .8]
    t, cmd, q = _protocol_sweep(freqs, lambda f, local, c: (2.0 if f == .5 else .4) * c)
    result = fa.process_sweep_group(_group(t, cmd, q, {"freqs": freqs}), [])
    assert [p["freq_hz"] for p in result["points"]] == freqs
    assert abs(result["points"][0]["amp_ratio"] - 2.0) < .01
    assert abs(result["points"][1]["amp_ratio"] - .4) < .01


def test_segmented_sweep_drops_first_cycle_and_recovers_response():
    f = 1.2
    def response(_, local, c):
        good = .024 * np.sin(2*np.pi*f*local - .5)
        return np.where(local < 1/f, 4.0 * np.sin(2*np.pi*f*local), good)
    t, cmd, q = _protocol_sweep([f], response)
    point = fa.process_sweep_group(_group(t, cmd, q, {"freq_hz": f}, name="C/test/1.2Hz"), [])["points"][0]
    assert abs(point["amp_ratio"] - .8) < .02 and abs(point["phase_lag_deg"] - np.degrees(.5)) < 2, point
    assert point["n_cycles"] == 9 and point["window_start"] > 1.0


def test_segmented_sweep_residual_ratio_uses_response_amplitude():
    t = np.arange(0.0, 5.0, .002)
    f = 1.0
    cmd = .03 * np.sin(2*np.pi*f*t)
    q = .012 * np.sin(2*np.pi*f*t-.4) + .003 * np.sin(4*np.pi*f*t)
    point = fa._fit_sine_window(t, cmd, q, f, 0.0, 5.0, 5)
    assert abs(point["residual_ratio"] - point["residual_rms"] / .012) < 1e-3, point


def test_phase_unwrap_across_groups():
    points = [{"freq_hz": 1., "phase_lag_deg": 170.}, {"freq_hz": 2., "phase_lag_deg": -170.}]
    fa._unwrap_points(points)
    assert points[1]["phase_lag_unwrapped_deg"] == 190.0


def test_aggregate_quality_flags_and_model_selection():
    base = {"td": .06, "vmax": 6., "tau_m": .05, "K": 1., "nrmse": .03, "td_geom": .06,
            "rate_sat_frac": .1, "dq_peak": .5, "dc_gain": 1., "overshoot": 0., "steady_error": 0.,
            "optimizer": {"success": True}, "any_bound_hit": False, "rate_identifiable": False}
    agg = fa._aggregate([base, dict(base)], False)
    assert agg["pass_optimizer"] and agg["pass_no_bounds"] and agg["pass_all"]
    bad = dict(base, any_bound_hit=True); assert not fa._aggregate([bad], False)["pass_all"]
    unstable = [dict(base), dict(base, td=.12, nrmse=.30)]
    unstable_agg = fa._aggregate(unstable, False)
    assert not unstable_agg["pass_td_std"] and not unstable_agg["pass_nrmse"]
    results = [{"joint": "j", "amplitude": .03, "agg_by_dir": {"+": agg}, "agg": agg}]
    assert fa._approved_models(results, False)["j"]["K"] == 1.0


def test_summary_schema_v2_and_old_fields():
    assert fa.model_bode(1.0, .05, tau=.1, K=1.2)[0] > fa.model_bode(1.0, .05, tau=.1)[0]


def test_tables_report_model_parameters():
    with TemporaryDirectory() as directory:
        first_path = Path(directory) / "first.md"
        second_path = Path(directory) / "second.md"
        fa.write_table([], False, first_path, "test-run")
        fa.write_table([], True, second_path, "test-run")
        first = first_path.read_text(encoding="utf-8")
        second = second_path.read_text(encoding="utf-8")
    assert "v_max rad/s" in first and "τm ms" in first and "rate identifiable" in first
    assert "ωn rad/s" in second and "| ζ |" in second
    assert "rate_identifiable=true" not in second


def test_nrmse_zero_for_exact_match_and_nan_for_flat():
    values = np.array([0.0, 1.0, 2.0])
    assert fa.nrmse(values, values) == 0.0
    assert np.isnan(fa.nrmse(np.zeros(3), np.zeros(3)))


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"PASS {name}")
            except AssertionError as exc:
                fails += 1; print(f"FAIL {name}: {exc}")
    print(f"\n{'所有测试通过' if not fails else f'{fails} 个失败'}")
    raise SystemExit(bool(fails))
