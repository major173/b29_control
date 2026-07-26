"""fit_actuator_id 自检：用已知真值的合成阶跃/正弦验证反演精度。

不依赖 pytest，用带 numpy/scipy 的解释器直接跑：
  python3 scripts/delay/test_fit_actuator_id.py
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

_spec = importlib.util.spec_from_file_location(
    "fit_actuator_id", Path(__file__).resolve().parent / "fit_actuator_id.py")
fa = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fa)


def _step_cmd(t, t_edge, amp):
    return np.where(t >= t_edge, amp, 0.0)


def test_step_fit_recovers_first_order_params():
    """惯性主导（小幅值、v_max 不饱和）时应准确反演 τd 与 τm。"""
    dt = 0.02
    t = np.arange(0.0, 2.0, dt)
    cmd = _step_cmd(t, 0.3, 0.03)
    td_true, vmax_true, tau_true = 0.06, 6.0, 0.05
    q = fa.simulate(t, cmd, td_true, vmax_true, tau_true, 0.0)
    params, model = fa.fit_step_event(t, cmd, q, second_order=False)
    assert abs(params["td"] - td_true) < 0.015, params
    assert abs(params["tau_m"] - tau_true) < 0.015, params
    assert params["nrmse"] < 0.02, params
    assert fa.nrmse(q, model) < 0.02


def test_step_fit_recovers_vmax_when_rate_saturated():
    """大幅值时速率限制主导，v_max 应准确、恒斜率占比应高。"""
    dt = 0.02
    t = np.arange(0.0, 2.0, dt)
    cmd = _step_cmd(t, 0.3, 0.60)          # 幅值远大于 v_max*τ，必然饱和
    q = fa.simulate(t, cmd, 0.06, 3.0, 0.02, 0.0)
    params, _ = fa.fit_step_event(t, cmd, q, second_order=False)
    assert abs(params["vmax"] - 3.0) / 3.0 < 0.10, params
    assert params["rate_sat_frac"] > 0.5, params


def test_rate_sat_frac_low_when_inertia_dominates():
    """惯性主导时恒斜率占比应显著低于饱和情形（两者要能区分开）。"""
    dt = 0.005
    t = np.arange(0.0, 2.0, dt)
    cmd = _step_cmd(t, 0.3, 0.03)
    q = fa.simulate(t, cmd, 0.06, 60.0, 0.05, 0.0)   # v_max 抬高到不可能饱和
    params, _ = fa.fit_step_event(t, cmd, q, second_order=False)
    assert params["rate_sat_frac"] < 0.45, params     # 饱和情形 >0.8，两者可分


def test_step_fit_second_order_recovers_wn_zeta():
    dt = 0.005
    t = np.arange(0.0, 3.0, dt)
    cmd = _step_cmd(t, 0.5, 0.03)
    wn_true, zeta_true = 12.0, 0.35
    q = fa.simulate(t, cmd, 0.04, 1e3, None, 0.0, wn=wn_true, zeta=zeta_true)
    params, _ = fa.fit_step_event(t, cmd, q, second_order=True)
    assert abs(params["wn"] - wn_true) / wn_true < 0.15, params
    assert abs(params["zeta"] - zeta_true) < 0.08, params
    assert params["nrmse"] < 0.05, params


def test_td_geom_tracks_true_delay():
    dt = 0.002                              # 细采样下几何估计应贴近真值
    t = np.arange(0.0, 1.5, dt)
    cmd = _step_cmd(t, 0.3, 0.03)
    q = fa.simulate(t, cmd, 0.06, 6.0, 0.05, 0.0)
    params, _ = fa.fit_step_event(t, cmd, q, second_order=False)
    assert abs(params["td_geom"] - 0.06) < 0.012, params


def test_find_step_edges_counts_rising_and_falling():
    dt = 0.02
    t = np.arange(0.0, 6.0, dt)
    cmd = np.zeros_like(t)
    for k in range(2):                      # 两次 起跳→回落
        cmd[(t >= 1.0 + 2 * k) & (t < 2.0 + 2 * k)] = 0.03
    edges = fa.find_step_edges(t, cmd, 0.03)
    assert len(edges) == 4, edges
    for i0, e, i1 in edges:
        assert i0 < e < i1


def test_sine_fit_amplitude_and_phase():
    t = np.arange(0.0, 4.0, 0.01)
    amp, phase = fa.sine_fit(t, 0.7 * np.sin(2 * np.pi * 1.5 * t + 0.4) + 0.1, 1.5)
    assert abs(amp - 0.7) < 1e-6
    assert abs(phase - 0.4) < 1e-6


def test_sweep_points_matches_analytic_first_order():
    """扫频反演的幅值比/相位应与一阶+延迟解析式一致。"""
    dt = 0.002
    t = np.arange(0.0, 20.0, dt)
    f = 1.5
    cmd = 0.03 * np.sin(2 * np.pi * f * t)
    td, tau = 0.06, 0.05
    q = fa.simulate(t, cmd, td, 1e3, tau, 0.0)
    pts = fa.sweep_points(t, cmd, q, [f])
    mag, lag = fa.model_bode(f, td=td, tau=tau)
    # 跳过起始瞬态后比较
    n0 = int(2.0 / dt)
    pts = fa.sweep_points(t[n0:] - t[n0], cmd[n0:], q[n0:], [f])
    assert abs(pts[0]["amp_ratio"] - mag) < 0.03, (pts, mag)
    assert abs(pts[0]["phase_lag_deg"] - lag) < 5.0, (pts, lag)


def test_sweep_points_skips_absent_frequency():
    t = np.arange(0.0, 5.0, 0.01)
    cmd = 0.03 * np.sin(2 * np.pi * 0.5 * t)
    pts = fa.sweep_points(t, cmd, cmd.copy(), [0.5, 7.0])
    assert [p["freq_hz"] for p in pts] == [0.5]


def test_fit_falling_edge_recovers_params():
    """回落沿（amp→0）应与抬起沿同样准确反演，不能被当抬起沿硬拟合。"""
    dt = 0.02
    t = np.arange(0.0, 2.0, dt)
    cmd = np.where(t >= 0.3, 0.0, 0.03)     # 从 0.03 落回 0
    td_true, tau_true = 0.06, 0.05
    q = fa.simulate(t, cmd, td_true, 6.0, tau_true, float(cmd[0]))
    params, _ = fa.fit_step_event(t, cmd, q, second_order=False)
    assert abs(params["td"] - td_true) < 0.015, params
    assert abs(params["tau_m"] - tau_true) < 0.015, params
    assert params["nrmse"] < 0.02, params


def test_process_step_group_splits_by_direction():
    """agg_by_dir 应把抬起沿(+)与回落沿(-)分开统计，且方向标注正确。"""
    dt = 0.02
    t = np.arange(0.0, 6.0, dt)
    cmd1 = np.zeros_like(t)
    for k in range(2):                      # 两次 起跳→回落 = 2 个 + 沿、2 个 - 沿
        cmd1[(t >= 1.0 + 2 * k) & (t < 2.0 + 2 * k)] = 0.03
    q1 = fa.simulate(t, cmd1, 0.06, 6.0, 0.05, 0.0)
    n = len(t)
    g = {"name": "B/test/+0.03", "joint": "test_joint", "kind": "step", "j": 0,
         "t": t, "cmd": np.column_stack([cmd1] + [np.zeros(n)] * 3),
         "q": np.column_stack([q1] + [np.zeros(n)] * 3),
         "dq": np.zeros((n, 4)), "meta": {"amplitude": 0.03}}
    r = fa.process_step_group(g, second_order=False, outdir=".", make_plot=False)
    assert set(r["agg_by_dir"]) == {"+", "-"}, r["agg_by_dir"].keys()
    assert sum(1 for e in r["events"] if e["dir"] == "+") == 2, r["events"]
    assert sum(1 for e in r["events"] if e["dir"] == "-") == 2, r["events"]
    for e in r["events"]:                   # 两个方向都应拟合出同一组真值
        assert abs(e["td"] - 0.06) < 0.021, e
        assert e["nrmse"] < 0.05, e


def test_aggregate_pass_flags():
    events = [{"td": 0.060, "vmax": 6.0, "tau_m": 0.05, "nrmse": 0.03,
               "td_geom": 0.06, "rate_sat_frac": 0.1, "dq_peak": 0.5},
              {"td": 0.064, "vmax": 6.1, "tau_m": 0.05, "nrmse": 0.04,
               "td_geom": 0.06, "rate_sat_frac": 0.1, "dq_peak": 0.5}]
    agg = fa._aggregate(events, second_order=False)
    assert agg["pass_td_std"] and agg["pass_nrmse"]
    events[1]["td"] = 0.120                  # σ=30ms > 10ms
    events[1]["nrmse"] = 0.30
    agg = fa._aggregate(events, second_order=False)
    assert not agg["pass_td_std"] and not agg["pass_nrmse"]


def test_nrmse_zero_for_exact_match_and_nan_for_flat():
    x = np.array([0.0, 1.0, 2.0])
    assert fa.nrmse(x, x) == 0.0
    assert np.isnan(fa.nrmse(np.zeros(3), np.zeros(3)))


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if not name.startswith("test_") or not callable(fn):
            continue
        try:
            fn()
            print(f"PASS {name}")
        except AssertionError as exc:
            fails += 1
            print(f"FAIL {name}: {exc}")
    print(f"\n{'所有测试通过' if not fails else f'{fails} 个失败'}")
    raise SystemExit(1 if fails else 0)
