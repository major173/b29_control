#!/usr/bin/env python3
"""执行器辨识拟合：session 数据 → 局部增量 v2 阶跃/扫频报告。

输入是 actuator_id_node.py 产出的 group_*.npz；不读取 rosbag。输出 v2
summary.json、阶跃参数表和 Bode 表/图。前向链保持：纯延迟 → 速率限制 → 动态环节。
"""

import argparse
import ast
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.optimize import least_squares


# --------------------------------------------------------------------------- #
# 前向模型：局部命令增量的纯延迟 → 速率限制 → 一阶/二阶动态
# --------------------------------------------------------------------------- #

def simulate(t, cmd, td, vmax, tau, q0, wn=None, zeta=None, K=1.0):
    """按采样点前向仿真，保留旧调用签名，新增可选直流增益 ``K``。

    ``cmd`` 可为绝对命令（旧 API）或局部增量。内部总是以 ``cmd[0]`` 为
    命令基线，输出为 ``q0 + K * dynamic(cmd-cmd[0])``。
    """
    t = np.asarray(t, dtype=np.float64)
    cmd = np.asarray(cmd, dtype=np.float64)
    if len(t) < 2:
        return np.full_like(cmd, float(q0))
    dt = float(np.median(np.diff(t)))
    u = cmd - cmd[0]
    delayed = np.interp(t - td, t, u, left=0.0, right=float(u[-1]))

    ramp = np.empty_like(delayed)
    state = 0.0
    lim = float(vmax) * dt
    for i, target in enumerate(delayed):
        state += np.clip(target - state, -lim, lim)
        ramp[i] = state

    dynamic = np.empty_like(ramp)
    if wn is None:
        # 与旧实现相同的后向 Euler 一阶离散；仅状态改为局部增量。
        alpha = dt / (float(tau) + dt)
        state = 0.0
        for i, target in enumerate(ramp):
            state += alpha * (target - state)
            dynamic[i] = state
    else:
        # ZOH 精确离散。避免显式子步 Euler 在高 wn 下发散/overflow。
        wn, zeta = float(wn), float(zeta)
        augmented = np.array([[0.0, 1.0, 0.0],
                              [-wn * wn, -2.0 * zeta * wn, wn * wn],
                              [0.0, 0.0, 0.0]])
        transition = expm(augmented * dt)
        ad, bd = transition[:2, :2], transition[:2, 2]
        state = np.zeros(2)
        for i, target in enumerate(ramp):
            state = ad @ state + bd * target
            dynamic[i] = state[0]
    return float(q0) + float(K) * dynamic


def nrmse(q_meas, q_model):
    span = float(np.ptp(q_meas))
    if span < 1e-9:
        return float("nan")
    return float(np.sqrt(np.mean((q_meas - q_model) ** 2)) / span)


# --------------------------------------------------------------------------- #
# 阶跃事件
# --------------------------------------------------------------------------- #

def find_step_edges(t, cmd, amp, pre_s=0.3, post_s=None):
    """返回 ``(i0, i_edge, i1)`` 阶跃局部窗口。"""
    dt = float(np.median(np.diff(t)))
    jump = np.abs(np.diff(cmd)) > max(abs(amp) * 0.5, 1e-9)
    edges = np.flatnonzero(jump) + 1
    n_pre = int(round(pre_s / dt))
    out = []
    for k, edge in enumerate(edges):
        next_edge = edges[k + 1] if k + 1 < len(edges) else len(cmd)
        i1 = next_edge if post_s is None else min(next_edge, edge + int(round(post_s / dt)))
        i0 = max(0, edge - n_pre)
        if i1 - edge > int(round(0.5 / dt)):
            out.append((i0, edge, i1))
    return out


def _bound_hits(x, lo, hi):
    x, lo, hi = np.asarray(x), np.asarray(lo), np.asarray(hi)
    tol = 1e-6 + 1e-4 * (hi - lo)
    return (np.abs(x - lo) <= tol) | (np.abs(x - hi) <= tol)


def _local_baselines(t, cmd, q):
    edge = int(np.argmax(np.abs(np.diff(cmd))) + 1)
    pre = slice(0, max(edge, 1))
    return edge, float(np.median(cmd[pre])), float(np.median(q[pre]))


def fit_step_event(t, cmd, q, second_order=False):
    """在局部增量坐标拟合单个阶跃，并返回参数、质量诊断和模型。"""
    t = np.asarray(t, dtype=np.float64) - float(t[0])
    cmd, q = np.asarray(cmd, dtype=np.float64), np.asarray(q, dtype=np.float64)
    edge, cmd_pre, q_pre = _local_baselines(t, cmd, q)
    u = cmd - cmd_pre
    if second_order:
        names = ("td", "wn", "zeta", "K")
        p0, lo, hi = ([0.06, 12.0, 0.7, 1.0], [0.0, 0.5, 0.05, 0.1],
                      [0.30, 200.0, 30.0, 3.0])

        def forward(p):
            return simulate(t, u, p[0], 1e6, None, q_pre, wn=p[1], zeta=p[2], K=p[3])
    else:
        names = ("td", "vmax", "tau_m", "K")
        p0, lo, hi = ([0.06, 6.0, 0.05, 1.0], [0.0, 0.2, 0.002, 0.1],
                      [0.30, 60.0, 0.60, 3.0])

        def forward(p):
            return simulate(t, u, p[0], p[1], p[2], q_pre, K=p[3])

    sol = least_squares(lambda p: forward(p) - q, p0, bounds=(lo, hi),
                        xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=2000)
    model = forward(sol.x)
    params = {name: float(value) for name, value in zip(names, sol.x)}
    hits = _bound_hits(sol.x, lo, hi)
    optimizer = {"success": bool(sol.success), "status": int(sol.status),
                 "message": str(sol.message), "cost": float(sol.cost),
                 "optimality": float(sol.optimality), "nfev": int(sol.nfev),
                 "active_mask": [int(x) for x in sol.active_mask]}
    params.update({
        "cmd_pre": cmd_pre,
        "q_pre": q_pre,
        "nrmse": nrmse(q, model),
        # 顶层字段方便 JSON 审计；optimizer 保留为结构化兼容别名。
        **optimizer,
        "optimizer": optimizer,
        "bound_hits": {name: bool(hit) for name, hit in zip(names, hits)},
        "any_bound_hit": bool(np.any(hits)),
    })

    command_delta = float(np.median(cmd[-max(1, len(cmd) // 8):]) - cmd_pre)
    q_tail = float(np.median(q[-max(1, len(q) // 8):]))
    response = q - q_pre
    signed = np.sign(command_delta) if abs(command_delta) > 1e-9 else 1.0
    final_delta = q_tail - q_pre
    params["dc_gain"] = float(final_delta / command_delta) if abs(command_delta) > 1e-9 else float("nan")
    params["overshoot"] = float(max(0.0, np.max(signed * response) - signed * final_delta))
    params["steady_error"] = float(q_tail - (q_pre + params["K"] * command_delta))

    threshold = max(abs(command_delta) * 0.02, 2e-4)
    moved = np.flatnonzero(np.abs(q[edge:] - q_pre) > threshold)
    params["td_geom"] = float(t[edge + moved[0]] - t[edge]) if len(moved) else float("nan")
    dq = np.gradient(q, t)
    peak = float(np.percentile(np.abs(dq), 98))
    moving = np.abs(dq) > 0.1 * peak
    params["rate_sat_frac"] = (float(np.mean(np.abs(dq[moving]) > 0.8 * peak))
                               if peak > 1e-6 and moving.any() else 0.0)
    params["dq_peak"] = peak
    params["rate_identifiable"] = bool(not second_order and params["rate_sat_frac"] > 0.6
                                        and not params["bound_hits"]["vmax"])
    params["fit_valid"] = bool(sol.success and np.isfinite(sol.cost) and np.isfinite(params["nrmse"])
                               and not params["any_bound_hit"])
    return params, model


# --------------------------------------------------------------------------- #
# 扫频：兼容 sweep_points；组处理严格按采集协议切段
# --------------------------------------------------------------------------- #

def _sine_design(t, f):
    centered = t - float(np.mean(t))
    w = 2.0 * np.pi * f
    return np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t), centered])


def sine_fit(t, x, f):
    """旧 API：返回已知频率上的 ``(amplitude, phase_rad)``。"""
    coef, *_ = np.linalg.lstsq(_sine_design(np.asarray(t), f), np.asarray(x), rcond=None)
    return float(np.hypot(coef[0], coef[1])), float(np.arctan2(coef[1], coef[0]))


def _fit_sine_window(t, cmd, q, f, window_start, window_end, n_cycles):
    mask = (t >= window_start) & (t < window_end)
    tt, cc, qq = t[mask], cmd[mask], q[mask]
    if len(tt) < 8:
        return None
    amp_cmd, phase_cmd = sine_fit(tt, cc, f)
    if amp_cmd < 1e-6:
        return None
    amp_q, phase_q = sine_fit(tt, qq, f)
    pred = _sine_design(tt, f) @ np.linalg.lstsq(_sine_design(tt, f), qq, rcond=None)[0]
    lag = float(np.degrees(phase_cmd - phase_q))
    wrapped = (lag + 180.0) % 360.0 - 180.0
    rms = float(np.sqrt(np.mean((qq - pred) ** 2)))
    return {"freq_hz": float(f), "amp_cmd": amp_cmd, "amp_ratio": float(amp_q / amp_cmd),
            "phase_lag_deg": wrapped, "phase_lag_unwrapped_deg": wrapped,
            "window_start": float(window_start), "window_end": float(window_end),
            "n_samples": int(len(tt)), "n_cycles": float(n_cycles),
            "residual_rms": rms,
            "residual_ratio": float(rms / amp_q) if amp_q > 1e-9 else float("nan")}


def sweep_points(t, cmd, q, freqs):
    """旧 API：整窗单频拟合，供旧调用方与兼容测试使用。"""
    pts = []
    for f in freqs:
        amp_c, phase_c = sine_fit(t, cmd, f)
        if amp_c < 1e-3:
            continue
        amp_q, phase_q = sine_fit(t, q, f)
        lag = (np.degrees(phase_c - phase_q) + 180.0) % 360.0 - 180.0
        pts.append({"freq_hz": float(f), "amp_ratio": float(amp_q / amp_c),
                    "phase_lag_deg": float(lag), "amp_cmd": float(amp_c)})
    return pts


def _unwrap_points(points):
    if points:
        order = np.argsort([p["freq_hz"] for p in points])
        vals = np.unwrap(np.radians([points[i]["phase_lag_deg"] for i in order]))
        for i, value in zip(order, np.degrees(vals)):
            points[i]["phase_lag_unwrapped_deg"] = float(value)


def model_bode(f, td, tau=None, wn=None, zeta=None, K=1.0):
    """带直流增益 K 的一阶/二阶连续解析 Bode 点。"""
    w = 2.0 * np.pi * f
    if wn is None:
        h = 1.0 / (1.0 + 1j * w * tau)
    else:
        h = wn ** 2 / (wn ** 2 - w ** 2 + 2j * zeta * wn * w)
    h = float(K) * h * np.exp(-1j * w * td)
    return float(np.abs(h)), float(-np.degrees(np.angle(h)))


# --------------------------------------------------------------------------- #
# 组遍历与聚合
# --------------------------------------------------------------------------- #

def load_groups(rundir):
    groups = []
    for path in sorted(Path(rundir).glob("group_*.npz")):
        data = np.load(path, allow_pickle=True)
        groups.append({"path": path, "name": str(data["name"]), "joint": str(data["joint"]),
                       "kind": str(data["kind"]), "j": int(data["joint_index"]),
                       "t": np.asarray(data["t"], dtype=np.float64),
                       "cmd": np.asarray(data["cmd"], dtype=np.float64),
                       "q": np.asarray(data["q"], dtype=np.float64),
                       "dq": np.asarray(data["dq"], dtype=np.float64),
                       "meta": ast.literal_eval(str(data["meta"])) if "meta" in data else {}})
    if not groups:
        raise SystemExit(f"{rundir} 下没有 group_*.npz")
    return groups


def process_step_group(g, second_order, outdir, make_plot=True):
    j = g["j"]
    t, cmd, q = g["t"], g["cmd"][:, j], g["q"][:, j]
    amp = float(g["meta"].get("amplitude", 0.0)) or float(np.max(np.abs(np.diff(cmd))))
    events, overlays = [], []
    for i0, edge, i1 in find_step_edges(t, cmd, amp):
        params, model = fit_step_event(t[i0:i1], cmd[i0:i1], q[i0:i1], second_order)
        params["dir"] = "+" if float(cmd[edge] - cmd[edge - 1]) > 0 else "-"
        params["t_edge"] = float(t[edge] - t[0])
        events.append(params)
        overlays.append((t[i0:i1] - t[i0], cmd[i0:i1], q[i0:i1], model, params["dir"]))
    if make_plot and overlays:
        _plot_step_fit(g, overlays, outdir)
    by_dir = {d: [event for event in events if event["dir"] == d] for d in ("+", "-")}
    return {"group": g["name"], "joint": g["joint"], "kind": "step", "amplitude": amp,
            "n_events": len(events), "events": events,
            "agg_by_dir": {d: _aggregate(events, second_order) for d, events in by_dir.items() if events},
            "agg": _aggregate(events, second_order)}


def _aggregate(events, second_order):
    if not events:
        return {}
    keys = (["td", "wn", "zeta", "K"] if second_order
            else ["td", "vmax", "tau_m", "K"])
    agg = {}
    for key in keys + ["nrmse", "td_geom", "rate_sat_frac", "dq_peak", "dc_gain", "overshoot", "steady_error"]:
        values = np.array([e[key] for e in events if np.isfinite(e.get(key, np.nan))])
        if len(values):
            agg[key] = {"mean": float(values.mean()), "std": float(values.std()),
                        "min": float(values.min()), "max": float(values.max())}
    td = agg.get("td", {})
    agg["pass_td_std"] = bool(td.get("std", 1.0) < 0.010)
    agg["pass_nrmse"] = bool(agg.get("nrmse", {}).get("mean", 1.0) < 0.10)
    agg["pass_optimizer"] = bool(all(e.get("optimizer", {}).get("success", False) for e in events))
    agg["pass_no_bounds"] = bool(not any(e.get("any_bound_hit", True) for e in events))
    agg["pass_all"] = bool(agg["pass_td_std"] and agg["pass_nrmse"]
                           and agg["pass_optimizer"] and agg["pass_no_bounds"])
    agg["rate_identifiable"] = bool(any(e.get("rate_identifiable", False) for e in events))
    return agg


def process_sweep_group(g, freqs_hint):
    """严格按 actuator_id_node 的 1s hold/周期协议切分，丢弃每段首周期。"""
    j = g["j"]
    t = np.asarray(g["t"], dtype=np.float64) - float(g["t"][0])
    cmd, q, meta = g["cmd"][:, j], g["q"][:, j], g["meta"]
    freqs = ([float(meta["freq_hz"])] if "freq_hz" in meta
             else [float(f) for f in meta.get("freqs", freqs_hint)])
    cursor, points = 0.0, []
    for f in freqs:
        cycles = 6 if f < 1.0 else 10
        sine_start, sine_end = cursor + 1.0, cursor + 1.0 + cycles / f
        point = _fit_sine_window(t, cmd, q, f, sine_start + 1.0 / f, sine_end, cycles - 1)
        if point is not None:
            points.append(point)
        cursor = sine_end
    _unwrap_points(points)
    return {"group": g["name"], "joint": g["joint"], "kind": "sweep", "points": points}


def _unwrap_all_sweeps(results):
    by_joint = {}
    for result in results:
        by_joint.setdefault(result["joint"], []).extend(result["points"])
    for points in by_joint.values():
        _unwrap_points(points)


# --------------------------------------------------------------------------- #
# 图与报告
# --------------------------------------------------------------------------- #

def _plot_step_fit(g, overlays, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(overlays), figsize=(3.2 * len(overlays), 3.0), squeeze=False, sharey=True)
    for ax, (tt, cc, qq, mm, direction) in zip(axes[0], overlays):
        ax.plot(tt, cc, "k--", lw=1.0, label="cmd")
        ax.plot(tt, qq, "C0", lw=1.2, label="q measured")
        ax.plot(tt, mm, "C3", lw=1.0, label="K·model")
        ax.set_title(f"edge {direction}", fontsize=8); ax.set_xlabel("t [s]"); ax.grid(alpha=0.3)
    axes[0][0].set_ylabel("position [rad]"); axes[0][0].legend(fontsize=7)
    fig.suptitle(f"{g['name']} local-increment step fit", fontsize=9); fig.tight_layout()
    fig.savefig(Path(outdir) / f"fit_{g['name'].replace('/', '_')}.png", dpi=110); plt.close(fig)


def _plot_bode(joint, pts, model, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pts = sorted(pts, key=lambda point: point["freq_hz"])
    f = np.array([p["freq_hz"] for p in pts]); mag = np.array([p["amp_ratio"] for p in pts])
    wrapped = np.array([p["phase_lag_deg"] for p in pts]); unwrapped = np.array([p["phase_lag_unwrapped_deg"] for p in pts])
    residual = np.array([p["residual_ratio"] for p in pts])
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(5.4, 6.4), sharex=True)
    a1.semilogx(f, mag, "C0o-", label="measured")
    a2.semilogx(f, wrapped, "C0o-", label="wrapped")
    a2.semilogx(f, unwrapped, "C2s--", label="unwrapped")
    a3.semilogx(f, residual, "C1o-", label="fit residual / response amplitude")
    if model is not None:
        fm = np.geomspace(max(f.min(), 0.05), f.max() * 1.2, 120); mm = [model_bode(x, **model) for x in fm]
        a1.semilogx(fm, [m[0] for m in mm], "C3--", label="approved step model")
        a2.semilogx(fm, [m[1] for m in mm], "C3--", label="model phase")
    for p in pts:
        a3.annotate(f"{p['n_cycles']:g} cyc", (p["freq_hz"], p["residual_ratio"]), fontsize=6)
    a1.axhline(1.0, color="gray", lw=0.6)
    for ax in (a1, a2, a3): ax.grid(alpha=0.3, which="both"); ax.legend(fontsize=7)
    a1.set_ylabel("amplitude ratio"); a2.set_ylabel("phase lag [deg]"); a3.set_ylabel("residual ratio"); a3.set_xlabel("f [Hz]")
    fig.suptitle(f"{joint} Bode (segmented test C)", fontsize=9); fig.tight_layout()
    fig.savefig(Path(outdir) / f"bode_{joint}.png", dpi=110); plt.close(fig)


def _f(agg, key, scale=1.0, fmt="{:.2f}±{:.2f}"):
    value = agg.get(key)
    return "-" if not value else fmt.format(value["mean"] * scale, value["std"] * scale)


def _mark(value):
    return "PASS" if value else "FAIL"


def write_table(step_results, second_order, path, rundir):
    dynamic = "二阶 (ωn, ζ)" if second_order else "一阶 τm"
    lines = ["# 执行器辨识参数表（v2，测试 B）", "", f"数据来源：`{rundir}`", "",
             f"局部增量模型：q_pre + K·(纯延迟 τd → 速率限制 → {dynamic})。",
             "`pass_all` = τd σ、NRMSE、优化器成功且无边界命中均通过；触边界或优化失败不可批准。", ""]
    columns = ["组", "关节", "沿", "n", "τd ms", "K"]
    if second_order:
        columns += ["ωn rad/s", "ζ"]
    else:
        columns += ["v_max rad/s", "τm ms", "rate identifiable"]
    columns += ["实测 dc_gain", "overshoot rad", "steady_error rad", "NRMSE",
                "optimizer", "bounds", "pass_all"]
    lines += ["| " + " | ".join(columns) + " |",
              "|" + "|".join(["---"] * len(columns)) + "|"]
    for result in step_results:
        for direction, agg in sorted(result.get("agg_by_dir", {}).items()):
            n = sum(e["dir"] == direction for e in result["events"])
            values = [result["group"], result["joint"], direction, str(n),
                      _f(agg, "td", 1e3, "{:.1f}±{:.1f}"), _f(agg, "K")]
            if second_order:
                values += [_f(agg, "wn"), _f(agg, "zeta")]
            else:
                values += [_f(agg, "vmax"), _f(agg, "tau_m", 1e3, "{:.1f}±{:.1f}"),
                           "YES" if agg.get("rate_identifiable") else "NO"]
            values += [_f(agg, "dc_gain"), _f(agg, "overshoot"), _f(agg, "steady_error"),
                       _f(agg, "nrmse", 100.0, "{:.1f}±{:.1f}%"),
                       _mark(agg.get("pass_optimizer")), _mark(agg.get("pass_no_bounds")),
                       _mark(agg.get("pass_all"))]
            lines.append("| " + " | ".join(values) + " |")
    if not second_order:
        lines += ["", "一阶中只有 `rate_sat_frac > 0.6` 且 `vmax` 未触边界时 `rate_identifiable=true`；",
                  "否则 v_max 仅作事件诊断，绝不进入批准代表模型。"]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def write_bode(sweep_results, model_by_joint, path, rundir):
    lines = ["# 执行器频域响应（v2，测试 C）", "", f"数据来源：`{rundir}`。每频点丢弃前 1 秒 hold 和首周期，",
             "在剩余稳态段最小二乘拟合 `[sin, cos, constant, linear trend]`。", "",
             "| 关节 | f Hz | amp_cmd | amp_ratio | phase wrapped deg | phase unwrapped deg | cycles | residual RMS | residual ratio | model amp | model phase |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for result in sweep_results:
        model = model_by_joint.get(result["joint"])
        for point in sorted(result["points"], key=lambda item: item["freq_hz"]):
            pred = model_bode(point["freq_hz"], **model) if model else (float("nan"), float("nan"))
            lines.append(f"| {result['joint']} | {point['freq_hz']:.2f} | {point['amp_cmd']:.4f} | {point['amp_ratio']:.3f} | {point['phase_lag_deg']:.1f} | {point['phase_lag_unwrapped_deg']:.1f} | {point['n_cycles']:g} | {point['residual_rms']:.5f} | {point['residual_ratio']:.3f} | {pred[0]:.3f} | {pred[1]:.1f} |")
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def _approved_models(step_results, second_order):
    models = {}
    for result in sorted(step_results, key=lambda item: abs(item["amplitude"])):
        aggregate = result.get("agg_by_dir", {}).get("+") or result.get("agg", {})
        if result["joint"] in models or not aggregate.get("pass_all", False):
            continue
        if second_order:
            models[result["joint"]] = {"td": aggregate["td"]["mean"], "tau": None,
                                        "wn": aggregate["wn"]["mean"], "zeta": aggregate["zeta"]["mean"],
                                        "K": aggregate["K"]["mean"]}
        else:
            models[result["joint"]] = {"td": aggregate["td"]["mean"], "tau": aggregate["tau_m"]["mean"],
                                        "K": aggregate["K"]["mean"]}
            if aggregate.get("rate_identifiable", False):
                models[result["joint"]]["vmax"] = aggregate["vmax"]["mean"]
    return models


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rundir", required=True, help="actuator_id_node 的 session 目录")
    parser.add_argument("--outdir", required=True)
    parser.add_argument("--second-order", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)
    step_results, sweep_results = [], []
    for group in load_groups(args.rundir):
        if group["kind"] == "step":
            step_results.append(process_step_group(group, args.second_order, outdir, not args.no_plots))
        else:
            sweep_results.append(process_sweep_group(group, []))
    _unwrap_all_sweeps(sweep_results)
    models = _approved_models(step_results, args.second_order)
    write_table(step_results, args.second_order, outdir / "actuator-id-table.md", args.rundir)
    if sweep_results:
        write_bode(sweep_results, models, outdir / "bode.md", args.rundir)
        if not args.no_plots:
            by_joint = {}
            for result in sweep_results: by_joint.setdefault(result["joint"], []).extend(result["points"])
            for joint, points in by_joint.items():
                if points: _plot_bode(joint, points, models.get(joint), outdir)
    summary = {"schema_version": 2, "model_description": "q_pre + K * dynamic(cmd-cmd_pre); delay → rate limit → first/second order", "rundir": str(args.rundir), "second_order": args.second_order, "step": step_results, "sweep": sweep_results, "model_by_joint": models}
    (outdir / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"阶跃组 {len(step_results)}，扫频组 {len(sweep_results)} → {outdir}")


if __name__ == "__main__":
    main()
