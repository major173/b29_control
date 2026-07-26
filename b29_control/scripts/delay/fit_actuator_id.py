#!/usr/bin/env python3
"""执行器辨识拟合（01 文档测试 B/C）：session 数据 → (τd, τm 或 ωn/ζ, v_max) 参数表。

输入是实机激励脚本（../actuator_id_node.py）产出的 session 目录，
里面每组一个 group_*.npz（字段 t/cmd/q/dq/joint/joint_index/kind/name）。
session.bag 只作归档，本工具不依赖 rosbag，用带 numpy/scipy 的解释器跑。

用法：
  python3 scripts/delay/fit_actuator_id.py \
      --rundir ~/actuator_id_runs/<stamp> \
      --outdir ~/actuator_id_runs/<stamp>/fit/

输出：
  actuator-id-table.md   关节 × 方向 × 幅值 → (τd, τm, v_max) + σ + NRMSE（测试 B 参数表）
  bode.md                关节 × 频率 → 幅值比 / 相位滞后（测试 C），附模型预测对比
  summary.json           机器可读全量结果（含每次阶跃的逐事件拟合）
  fit_<group>.png        每组阶跃拟合叠图；bode_<joint>.png 幅频/相频图

判据（01 测试 B）：组内 τd 标准差 < 10ms；模型复现 NRMSE < 10%。
"""

import argparse
import ast
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


# --------------------------------------------------------------------------- #
# 前向模型：纯延迟 → 速率限制 → 一阶惯性 / 二阶                                  #
# 与 RL 仓库 sim2deploy/gp11_reach/injection_chain.py 的注入链同构（顺序一致）    #
# --------------------------------------------------------------------------- #

def simulate(t, cmd, td, vmax, tau, q0, wn=None, zeta=None):
    """按 t 采样点前向仿真。tau 为一阶时间常数；给 wn/zeta 则改二阶。"""
    dt = float(np.median(np.diff(t)))
    delayed = np.interp(t - td, t, cmd, left=cmd[0], right=cmd[-1])
    # 速率限制（对延迟后的目标做斜坡跟随）
    ramp = np.empty_like(delayed)
    state = q0
    lim = vmax * dt
    for i, target in enumerate(delayed):
        state += np.clip(target - state, -lim, lim)
        ramp[i] = state
    # 惯性环节
    q = np.empty_like(ramp)
    if wn is None:
        alpha = dt / (tau + dt)
        pos = q0
        for i, target in enumerate(ramp):
            pos += alpha * (target - pos)
            q[i] = pos
    else:
        pos, vel = q0, 0.0
        sub, h = 4, dt / 4.0
        for i, target in enumerate(ramp):
            for _ in range(sub):
                acc = wn * wn * (target - pos) - 2.0 * zeta * wn * vel
                vel += acc * h
                pos += vel * h
            q[i] = pos
    return q


def nrmse(q_meas, q_model):
    span = float(np.ptp(q_meas))
    if span < 1e-9:
        return float("nan")
    return float(np.sqrt(np.mean((q_meas - q_model) ** 2)) / span)


# --------------------------------------------------------------------------- #
# 测试 B：阶跃事件切分与拟合                                                     #
# --------------------------------------------------------------------------- #

def find_step_edges(t, cmd, amp, pre_s=0.3, post_s=None):
    """返回 [(i0, i_edge, i1), ...]：每个阶跃沿前后各留一段窗口。"""
    dt = float(np.median(np.diff(t)))
    jump = np.abs(np.diff(cmd)) > abs(amp) * 0.5
    edges = np.flatnonzero(jump) + 1
    n_pre = int(round(pre_s / dt))
    out = []
    for k, e in enumerate(edges):
        nxt = edges[k + 1] if k + 1 < len(edges) else len(cmd)
        i1 = nxt if post_s is None else min(nxt, e + int(round(post_s / dt)))
        i0 = max(0, e - n_pre)
        if i1 - e > int(round(0.5 / dt)):        # 至少 0.5s 响应段才可拟合
            out.append((i0, e, i1))
    return out


def fit_step_event(t, cmd, q, second_order=False):
    """单个阶跃事件拟合。返回 dict（含参数、NRMSE、τd 的几何估计）。"""
    t = t - t[0]
    q0 = float(q[0])

    def resid(p):
        if second_order:
            td, wn, zeta = abs(p[0]), abs(p[1]), abs(p[2])
            model = simulate(t, cmd, td, 1e3, None, q0, wn=wn, zeta=zeta)
        else:
            td, vmax, tau = abs(p[0]), abs(p[1]), abs(p[2])
            model = simulate(t, cmd, td, vmax, tau, q0)
        return model - q

    p0 = [0.06, 12.0, 3.0] if second_order else [0.06, 6.0, 0.05]
    lo = [0.0, 0.5, 0.2] if second_order else [0.0, 0.2, 0.002]
    hi = [0.30, 200.0, 30.0] if second_order else [0.30, 60.0, 0.60]
    sol = least_squares(resid, p0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10)
    p = np.abs(sol.x)
    if second_order:
        model = simulate(t, cmd, p[0], 1e3, None, q0, wn=p[1], zeta=p[2])
        params = {"td": float(p[0]), "wn": float(p[1]), "zeta": float(p[2])}
    else:
        model = simulate(t, cmd, p[0], p[1], p[2], q0)
        params = {"td": float(p[0]), "vmax": float(p[1]), "tau_m": float(p[2])}
    params["nrmse"] = nrmse(q, model)

    # τd 的独立几何估计：命令跳变到 q 越过 2% 幅值的时间（与拟合值交叉验证）
    amp = float(cmd[-1] - cmd[0])
    i_edge = int(np.argmax(np.abs(np.diff(cmd))) + 1)
    thresh = max(abs(amp) * 0.02, 2e-4)
    moved = np.flatnonzero(np.abs(q[i_edge:] - q0) > thresh)
    params["td_geom"] = float(t[i_edge + moved[0]] - t[i_edge]) if len(moved) else float("nan")
    # 恒斜率占比：运动样本中"接近峰值速度"的比例（分母只算运动段，静置段不稀释）。
    # 速率限制主导 → 上升段 dq 近似恒定 → 接近 1；一阶惯性主导 → dq 指数衰减 → ~0.1
    dq = np.gradient(q, t)
    peak = float(np.percentile(np.abs(dq), 98))
    moving = np.abs(dq) > 0.1 * peak
    params["rate_sat_frac"] = (
        float(np.mean(np.abs(dq[moving]) > 0.8 * peak)) if peak > 1e-6 and moving.any() else 0.0)
    params["dq_peak"] = peak
    return params, model


# --------------------------------------------------------------------------- #
# 测试 C：单频最小二乘（幅值比 / 相位滞后）                                       #
# --------------------------------------------------------------------------- #

def sine_fit(t, x, f):
    """在已知频率 f 上拟合 x ≈ a·sin + b·cos + c，返回 (幅值, 相位 rad)。"""
    w = 2.0 * np.pi * f
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    a, b = coef[0], coef[1]
    return float(np.hypot(a, b)), float(np.arctan2(b, a))


def sweep_points(t, cmd, q, freqs):
    """对每个频点截出命令确有该频率成分的窗口，返回逐频点幅值比/相位滞后。"""
    pts = []
    for f in freqs:
        amp_c, ph_c = sine_fit(t, cmd, f)
        # 命令幅值太小说明该窗口不含此频率（合成组里其它频段），跳过
        if amp_c < 1e-3:
            continue
        amp_q, ph_q = sine_fit(t, q, f)
        lag = np.degrees(ph_c - ph_q)
        while lag < -180.0:
            lag += 360.0
        while lag > 180.0:
            lag -= 360.0
        pts.append({"freq_hz": float(f), "amp_ratio": amp_q / amp_c,
                    "phase_lag_deg": float(lag), "amp_cmd": amp_c})
    return pts


def model_bode(f, td, tau, wn=None, zeta=None):
    """一阶（或二阶）+ 纯延迟的解析 Bode 点，用于与测试 C 交叉验证。"""
    w = 2.0 * np.pi * f
    if wn is None:
        h = 1.0 / (1.0 + 1j * w * tau)
    else:
        h = wn ** 2 / (wn ** 2 - w ** 2 + 2j * zeta * wn * w)
    h = h * np.exp(-1j * w * td)
    return float(np.abs(h)), float(-np.degrees(np.angle(h)))


# --------------------------------------------------------------------------- #
# 组遍历与聚合                                                                  #
# --------------------------------------------------------------------------- #

def load_groups(rundir):
    groups = []
    for p in sorted(Path(rundir).glob("group_*.npz")):
        d = np.load(p, allow_pickle=True)
        groups.append({
            "path": p,
            "name": str(d["name"]),
            "joint": str(d["joint"]),
            "kind": str(d["kind"]),
            "j": int(d["joint_index"]),
            "t": np.asarray(d["t"], dtype=np.float64),
            "cmd": np.asarray(d["cmd"], dtype=np.float64),
            "q": np.asarray(d["q"], dtype=np.float64),
            "dq": np.asarray(d["dq"], dtype=np.float64),
            "meta": ast.literal_eval(str(d["meta"])) if "meta" in d else {},
        })
    if not groups:
        raise SystemExit(f"{rundir} 下没有 group_*.npz")
    return groups


def process_step_group(g, second_order, outdir, make_plot=True):
    j = g["j"]
    t, cmd, q = g["t"], g["cmd"][:, j], g["q"][:, j]
    amp = float(g["meta"].get("amplitude", 0.0)) or float(np.max(np.abs(np.diff(cmd))))
    events = []
    overlays = []
    for i0, e, i1 in find_step_edges(t, cmd, amp):
        seg_t, seg_cmd, seg_q = t[i0:i1], cmd[i0:i1], q[i0:i1]
        params, model = fit_step_event(seg_t, seg_cmd, seg_q, second_order)
        # 按沿的实际运动方向分组统计（01 测试 B 的"方向不对称"分支要用）。
        # 注意：不能用 |cmd末-cmd初| 判方向——抬起沿与回落沿的幅值绝对值相同。
        params["dir"] = "+" if float(cmd[e] - cmd[e - 1]) > 0 else "-"
        params["t_edge"] = float(t[e] - t[0])
        events.append(params)
        overlays.append((seg_t - seg_t[0], seg_cmd, seg_q, model, params["dir"]))
    if make_plot and overlays:
        _plot_step_fit(g, overlays, outdir)
    by_dir = {d: [e for e in events if e["dir"] == d] for d in ("+", "-")}
    return {"group": g["name"], "joint": g["joint"], "kind": "step",
            "amplitude": amp, "n_events": len(events), "events": events,
            "agg_by_dir": {d: _aggregate(evs, second_order)
                           for d, evs in by_dir.items() if evs},
            "agg": _aggregate(events, second_order)}


def _aggregate(events, second_order):
    if not events:
        return {}
    keys = ["td", "wn", "zeta"] if second_order else ["td", "vmax", "tau_m"]
    agg = {}
    for k in keys + ["nrmse", "td_geom", "rate_sat_frac", "dq_peak"]:
        vals = np.array([e[k] for e in events if not np.isnan(e.get(k, np.nan))])
        if len(vals):
            agg[k] = {"mean": float(vals.mean()), "std": float(vals.std()),
                      "min": float(vals.min()), "max": float(vals.max())}
    td = agg.get("td", {})
    agg["pass_td_std"] = bool(td.get("std", 1.0) < 0.010)          # < 10ms
    agg["pass_nrmse"] = bool(agg.get("nrmse", {}).get("mean", 1.0) < 0.10)
    return agg


def process_sweep_group(g, freqs_hint):
    j = g["j"]
    t, cmd, q = g["t"], g["cmd"][:, j], g["q"][:, j]
    meta = g["meta"]
    freqs = ([float(meta["freq_hz"])] if "freq_hz" in meta
             else [float(f) for f in meta.get("freqs", freqs_hint)])
    pts = sweep_points(t - t[0], cmd, q, freqs)
    return {"group": g["name"], "joint": g["joint"], "kind": "sweep", "points": pts}


# --------------------------------------------------------------------------- #
# 绘图                                                                         #
# --------------------------------------------------------------------------- #

def _plot_step_fit(g, overlays, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = len(overlays)
    fig, axes = plt.subplots(1, n, figsize=(3.2 * n, 3.0), squeeze=False, sharey=True)
    for ax, (tt, cc, qq, mm, d) in zip(axes[0], overlays):
        ax.plot(tt, cc, "k--", lw=1.0, label="cmd")
        ax.plot(tt, qq, "C0", lw=1.2, label="q measured")
        ax.plot(tt, mm, "C3", lw=1.0, label="model")
        ax.set_title(f"edge {d}", fontsize=8)
        ax.set_xlabel("t [s]")
        ax.grid(alpha=0.3)
    axes[0][0].set_ylabel("position [rad]")
    axes[0][0].legend(fontsize=7)
    fig.suptitle(f"{g['name']} step fit", fontsize=9)
    fig.tight_layout()
    out = Path(outdir) / f"fit_{g['name'].replace('/', '_')}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)


def _plot_bode(joint, pts, model, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    f = np.array([p["freq_hz"] for p in pts])
    order = np.argsort(f)
    f = f[order]
    mag = np.array([p["amp_ratio"] for p in pts])[order]
    lag = np.array([p["phase_lag_deg"] for p in pts])[order]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(5.2, 5.0), sharex=True)
    a1.semilogx(f, mag, "C0o-", label="measured")
    a2.semilogx(f, lag, "C0o-", label="measured")
    if model is not None:
        fm = np.geomspace(max(f.min(), 0.05), f.max() * 1.2, 120)
        mm = [model_bode(x, **model) for x in fm]
        a1.semilogx(fm, [m[0] for m in mm], "C3--", label="step-fit model")
        a2.semilogx(fm, [m[1] for m in mm], "C3--", label="step-fit model")
    a1.axhline(1.0, color="gray", lw=0.6)
    a1.set_ylabel("amplitude ratio")
    a1.grid(alpha=0.3, which="both")
    a1.legend(fontsize=7)
    a2.set_ylabel("phase lag [deg]")
    a2.set_xlabel("f [Hz]")
    a2.grid(alpha=0.3, which="both")
    fig.suptitle(f"{joint} Bode (test C)", fontsize=9)
    fig.tight_layout()
    fig.savefig(Path(outdir) / f"bode_{joint}.png", dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------- #
# 报告                                                                         #
# --------------------------------------------------------------------------- #

def _f(agg, key, scale=1.0, fmt="{:.1f}±{:.1f}"):
    d = agg.get(key)
    if not d:
        return "-"
    return fmt.format(d["mean"] * scale, d["std"] * scale)


def write_table(step_results, second_order, path, rundir):
    lines = [
        "# 执行器辨识参数表（01 测试 B）",
        "",
        f"数据来源：`{rundir}`　拟合工具：`scripts/delay/fit_actuator_id.py`",
        f"模型：纯延迟 τd → 速率限制 v_max → " +
        ("二阶 (ωn, ζ)" if second_order else "一阶 τm"),
        "",
        "判据：组内 τd 标准差 < 10ms、NRMSE < 10%（两列 ✅/❌）",
        "",
    ]
    if second_order:
        head = ("| 组 | 关节 | 指令幅值 rad | 沿 | n | τd ms | ωn rad/s | ζ | "
                "dq峰 rad/s | NRMSE | τd σ | NRMSE |")
        sep = "|---|---|---|---|---|---|---|---|---|---|---|---|"
    else:
        head = ("| 组 | 关节 | 指令幅值 rad | 沿 | n | τd ms | τd(几何) ms | v_max rad/s | "
                "τm ms | 恒斜率占比 | NRMSE | τd σ | NRMSE |")
        sep = "|---|---|---|---|---|---|---|---|---|---|---|---|---|"
    lines += [head, sep]
    for r in step_results:
        # 抬起沿与回落沿分行：同一组里两者混算会把方向不对称藏进 σ 里
        for d, a in sorted(r.get("agg_by_dir", {}).items()):
            if not a:
                continue
            n = sum(1 for e in r["events"] if e["dir"] == d)
            mark = lambda k: "✅" if a.get(k) else "❌"  # noqa: E731
            if second_order:
                cells = [r["group"], r["joint"], f"{r['amplitude']:+.3f}", d, str(n),
                         _f(a, "td", 1e3), _f(a, "wn"), _f(a, "zeta", 1.0, "{:.2f}±{:.2f}"),
                         _f(a, "dq_peak", 1.0, "{:.2f}±{:.2f}"),
                         _f(a, "nrmse", 100.0, "{:.1f}±{:.1f}%"),
                         mark("pass_td_std"), mark("pass_nrmse")]
            else:
                cells = [r["group"], r["joint"], f"{r['amplitude']:+.3f}", d, str(n),
                         _f(a, "td", 1e3), _f(a, "td_geom", 1e3),
                         _f(a, "vmax", 1.0, "{:.2f}±{:.2f}"), _f(a, "tau_m", 1e3),
                         _f(a, "rate_sat_frac", 1.0, "{:.2f}±{:.2f}"),
                         _f(a, "nrmse", 100.0, "{:.1f}±{:.1f}%"),
                         mark("pass_td_std"), mark("pass_nrmse")]
            lines.append("| " + " | ".join(cells) + " |")
    lines += [
        "",
        "## 读表要点",
        "",
        "- `τd` 是拟合值，`τd(几何)` 是命令跳变到 q 越过 2% 幅值的时间，两者应接近；",
        "  差异大说明一阶/速率限制的结构不对（考虑改二阶：`--second-order`）。",
        "- `沿` 列 `+`=离开基准位姿的抬起沿，`-`=回到基准位姿的回落沿。两者参数差异大",
        "  → 齿隙/摩擦方向性，走 01 测试 B 的「方向不对称」分支（模型分方向参数化）。",
        "  判据按沿分别评（混算会把方向差异藏进 σ 里）。",
        "- `恒斜率占比` >0.6 → 该幅值下速率限制主导，v_max 可信、τm 不可信；",
        "  <0.45 → 惯性主导，τm 可信而 **v_max 不可辨识**（拟合值会停在初值附近，只当下界看）。",
        "  按 01 测试 B 的分支：v_max 取大幅值组，τm 取小幅值组。",
        "- T3 的 DR 范围 = 实测均值 ± max(2σ, 20%)。",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


def write_bode(sweep_results, model_by_joint, path, rundir):
    lines = [
        "# 执行器频域响应（01 测试 C）",
        "",
        f"数据来源：`{rundir}`　单频最小二乘拟合（幅值比 = |q|/|cmd|）",
        "",
        "| 关节 | f Hz | 幅值比 | 相位滞后 deg | 模型幅值比 | 模型相位 deg |",
        "|---|---|---|---|---|---|",
    ]
    for r in sweep_results:
        m = model_by_joint.get(r["joint"])
        for p in sorted(r["points"], key=lambda x: x["freq_hz"]):
            mm = model_bode(p["freq_hz"], **m) if m else (float("nan"),) * 2
            lines.append(
                f"| {r['joint']} | {p['freq_hz']:.2f} | {p['amp_ratio']:.3f} | "
                f"{p['phase_lag_deg']:.1f} | {mm[0]:.3f} | {mm[1]:.1f} |")
    lines += [
        "",
        "判读（01 测试 C 分支）：幅值比在 1.5–2.2Hz 出现 >1 的谐振峰 → 二阶欠阻尼，",
        "T3 模型必须复现该谐振；高频相位滞后远超模型 → 有未建模高阶滞后，进测试 D。",
        "",
    ]
    Path(path).write_text("\n".join(lines), encoding="utf-8")


# --------------------------------------------------------------------------- #
# main                                                                         #
# --------------------------------------------------------------------------- #

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rundir", required=True, help="actuator_id_node 的 session 目录")
    p.add_argument("--outdir", required=True)
    p.add_argument("--second-order", action="store_true",
                   help="改用二阶 (ωn, ζ) 拟合（一阶拟合不上时，见 01 测试 B ❌ 分支）")
    p.add_argument("--no-plots", action="store_true")
    args = p.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    groups = load_groups(args.rundir)

    step_results, sweep_results = [], []
    for g in groups:
        if g["kind"] == "step":
            step_results.append(process_step_group(g, args.second_order, outdir,
                                                   make_plot=not args.no_plots))
        else:
            sweep_results.append(process_sweep_group(g, freqs_hint=[]))

    # 每个关节取小幅值组的模型作为 Bode 对比基准（惯性主导，参数最可信）
    model_by_joint = {}
    for r in sorted(step_results, key=lambda x: abs(x["amplitude"])):
        # 取抬起沿（+）的聚合；没有则退回全沿聚合
        a = r.get("agg_by_dir", {}).get("+") or r["agg"]
        if not a or r["joint"] in model_by_joint:
            continue
        if args.second_order:
            model_by_joint[r["joint"]] = {
                "td": a["td"]["mean"], "tau": None,
                "wn": a["wn"]["mean"], "zeta": a["zeta"]["mean"]}
        else:
            model_by_joint[r["joint"]] = {"td": a["td"]["mean"], "tau": a["tau_m"]["mean"]}

    write_table(step_results, args.second_order, outdir / "actuator-id-table.md", args.rundir)
    if sweep_results:
        write_bode(sweep_results, model_by_joint, outdir / "bode.md", args.rundir)
        if not args.no_plots:
            by_joint = {}
            for r in sweep_results:
                by_joint.setdefault(r["joint"], []).extend(r["points"])
            for joint, pts in by_joint.items():
                if pts:
                    _plot_bode(joint, pts, model_by_joint.get(joint), outdir)

    (outdir / "summary.json").write_text(json.dumps({
        "rundir": str(args.rundir),
        "second_order": args.second_order,
        "step": step_results,
        "sweep": sweep_results,
        "model_by_joint": model_by_joint,
    }, indent=2, ensure_ascii=False), encoding="utf-8")

    print(f"阶跃组 {len(step_results)}，扫频组 {len(sweep_results)} → {outdir}")
    key = "wn" if args.second_order else "tau_m"
    for r in step_results:
        if not r.get("agg_by_dir"):
            print(f"  {r['group']}: 无可用阶跃事件")
            continue
        for d, a in sorted(r["agg_by_dir"].items()):
            n = sum(1 for e in r["events"] if e["dir"] == d)
            print(f"  {r['group']} 沿{d}: n={n} τd={a['td']['mean']*1e3:.1f}±"
                  f"{a['td']['std']*1e3:.1f}ms {key}={a[key]['mean']:.4f} "
                  f"NRMSE={a['nrmse']['mean']*100:.1f}% "
                  f"[{'✅' if a['pass_td_std'] else '❌'}τdσ "
                  f"{'✅' if a['pass_nrmse'] else '❌'}NRMSE]")


if __name__ == "__main__":
    main()
