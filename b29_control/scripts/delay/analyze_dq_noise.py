#!/usr/bin/env python3
"""dq 观测噪声频谱分析与低通滤波强度定标（01 文档测试 F）。

两段式使用（rosbag 依赖与 scipy 依赖分属不同解释器）：

  # 1) 抽取（系统 python，需 rosbag）：
  env -i HOME=$HOME PATH=/usr/bin:/bin /usr/bin/python3 \
      scripts/delay/analyze_dq_noise.py extract \
      --bag <path.bag> --out <outdir>/<name>.npz

  # 2) 分析（conda python，需 numpy/scipy/matplotlib）：
  python3 scripts/delay/analyze_dq_noise.py analyze \
      --npz <outdir>/<name>.npz --outdir <outdir>/

分析内容：
  - /joint_states 时间戳抖动统计（05 文档 §7 callback 抖动检查）
  - 按运动幅度分段（static / slow / active），噪声只在对应段内估计
  - dq（驱动器上报）与 q 有限差分的对比：实机 dq 是否干净
  - 残差噪声频谱（Welch PSD）分段分关节
  - EMA 低通 α 扫描：降噪收益 vs 相位滞后代价 → 定标建议
"""

import argparse
import json
import sys
from pathlib import Path

LEG_JOINTS = [
    "left_first_leg_joint",
    "left_second_leg_joint",
    "right_first_leg_joint",
    "right_second_leg_joint",
]


# ---------------------------------------------------------------- extract

def cmd_extract(args):
    sys.path.insert(0, "/opt/ros/noetic/lib/python3/dist-packages")
    import numpy as np
    import rosbag

    bag_path = Path(args.bag)
    t_list, q_list, dq_list = [], [], []
    obs_t, obs_list = [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for _, msg, t_recv in bag.read_messages(topics=["/joint_states"]):
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            if not all(j in name_to_idx for j in LEG_JOINTS):
                continue
            idx = [name_to_idx[j] for j in LEG_JOINTS]
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = t_recv.to_sec()
            if len(msg.velocity) < max(idx) + 1:
                continue
            t_list.append(stamp)
            q_list.append([msg.position[i] for i in idx])
            dq_list.append([msg.velocity[i] for i in idx])
        for _, msg, t_recv in bag.read_messages(topics=["/gp11/rl/observation"]):
            obs_t.append(t_recv.to_sec())
            obs_list.append(list(msg.data))

    if not t_list:
        raise SystemExit("no usable /joint_states messages (missing leg joints or velocity)")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    kwargs = dict(
        t=np.asarray(t_list),
        q=np.asarray(q_list),
        dq=np.asarray(dq_list),
        joint_names=np.asarray(LEG_JOINTS),
        bag=str(bag_path),
    )
    if obs_list and len({len(o) for o in obs_list}) == 1:
        kwargs["obs_t"] = np.asarray(obs_t)
        kwargs["obs"] = np.asarray(obs_list)
    np.savez_compressed(out, **kwargs)
    print(f"saved {out}: {len(t_list)} joint_states samples, "
          f"{len(obs_list)} observation samples")


# ---------------------------------------------------------------- analyze

def ema(x, alpha):
    import numpy as np
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def ema_phase_lag_deg(alpha, f, dt):
    """一阶 EMA 在频率 f 处的相位滞后（度）。"""
    import numpy as np
    w = 2.0 * np.pi * f * dt
    z = np.exp(1j * w)
    h = alpha / (1.0 - (1.0 - alpha) * z ** -1)
    return float(-np.degrees(np.angle(h)))


def cmd_analyze(args):
    import numpy as np
    from scipy import signal as sig
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = np.load(args.npz, allow_pickle=False)
    t, q, dq = data["t"], data["q"], data["dq"]
    joint_names = [str(n) for n in data["joint_names"]]
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # -- 采样统计（callback 抖动）
    dts = np.diff(t)
    dt_med = float(np.median(dts))
    fs = 1.0 / dt_med
    jitter = {
        "dt_median_ms": dt_med * 1e3,
        "dt_p95_ms": float(np.percentile(dts, 95)) * 1e3,
        "dt_max_ms": float(dts.max()) * 1e3,
        "dt_std_ms": float(dts.std()) * 1e3,
    }

    # -- 重采样到均匀网格（谱分析前提）
    tu = np.arange(t[0], t[-1], dt_med)
    qu = np.stack([np.interp(tu, t, q[:, j]) for j in range(4)], axis=1)
    dqu = np.stack([np.interp(tu, t, dq[:, j]) for j in range(4)], axis=1)

    # -- 运动分段：零相位低通(3Hz)后的 |dq| 滚动 RMS，取关节最大值
    b, a = sig.butter(2, 3.0 / (fs / 2.0), "low")
    dq_smooth = sig.filtfilt(b, a, dqu, axis=0)
    win = max(int(round(args.seg_window * fs)), 5)
    kernel = np.ones(win) / win
    activity = np.sqrt(np.convolve((dq_smooth ** 2).max(axis=1), kernel, mode="same"))
    labels = np.full(len(tu), "slow", dtype=object)
    labels[activity < args.static_thresh] = "static"
    labels[activity >= args.active_thresh] = "active"

    # -- 噪声残差：dq − 零相位低通(5Hz)。零相位故不引入滞后，残差≈>5Hz 成分
    b5, a5 = sig.butter(2, args.noise_split_hz / (fs / 2.0), "low")
    resid = dqu - sig.filtfilt(b5, a5, dqu, axis=0)

    # -- q 有限差分速度（中心差分）与上报 dq 的对比
    dq_fd = np.gradient(qu, dt_med, axis=0)
    resid_fd = dq_fd - sig.filtfilt(b5, a5, dq_fd, axis=0)

    seg_stats = {}
    for cls in ("static", "slow", "active"):
        m = labels == cls
        if m.sum() < win:
            continue
        seg_stats[cls] = {
            "duration_s": float(m.sum() * dt_med),
            "dq_resid_rms": [float(np.sqrt((resid[m, j] ** 2).mean())) for j in range(4)],
            "fd_resid_rms": [float(np.sqrt((resid_fd[m, j] ** 2).mean())) for j in range(4)],
            "dq_signal_rms": [float(np.sqrt((dq_smooth[m, j] ** 2).mean())) for j in range(4)],
        }

    # -- EMA α 扫描：噪声衰减（static 段残差）vs 相位滞后（f_osc 处）
    alphas = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0]
    noise_mask = labels == ("static" if "static" in seg_stats else "slow")
    alpha_rows = []
    for al in alphas:
        dq_f = np.stack([ema(dqu[:, j], al) for j in range(4)], axis=1)
        resid_f = dq_f - sig.filtfilt(b5, a5, dq_f, axis=0)
        rms_ratio = float(
            np.sqrt((resid_f[noise_mask] ** 2).mean())
            / max(np.sqrt((resid[noise_mask] ** 2).mean()), 1e-12)
        )
        alpha_rows.append({
            "alpha": al,
            "tau_ms": dt_med * (1.0 - al) / al * 1e3 if al > 0 else float("inf"),
            "phase_lag_deg_at_fosc": ema_phase_lag_deg(al, args.fosc, dt_med),
            "noise_rms_ratio": rms_ratio,
        })

    # -- Welch PSD（分段分关节）
    cls_colors = {"static": "#4269d0", "slow": "#efb118", "active": "#ff725c"}
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True, sharey=True)
    for j, ax in enumerate(axes.ravel()):
        for cls in seg_stats:
            m = labels == cls
            nper = min(int(8 * fs), int(m.sum() // 2))
            if nper < int(2 * fs):
                continue
            f, pxx = sig.welch(dqu[m, j], fs=fs, nperseg=nper)
            ax.semilogy(f, pxx, color=cls_colors[cls], lw=1.5, label=cls)
        ax.axvline(args.fosc, color="0.6", ls="--", lw=1)
        ax.set_title(joint_names[j], fontsize=9)
        ax.grid(alpha=0.25, lw=0.5)
    axes[0, 0].legend(fontsize=8, frameon=False)
    for ax in axes[1, :]:
        ax.set_xlabel("frequency [Hz]")
    for ax in axes[:, 0]:
        ax.set_ylabel("dq PSD [(rad/s)$^2$/Hz]")
    fig.suptitle("dq PSD by motion segment (dashed = f_osc)")
    fig.tight_layout()
    fig.savefig(outdir / "psd_by_segment.png", dpi=150)
    plt.close(fig)

    # -- 时间线 + 分段
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    tt = tu - tu[0]
    for j in range(4):
        axes[0].plot(tt, dqu[:, j], lw=0.6, label=joint_names[j])
    axes[0].set_ylabel("dq [rad/s]")
    axes[0].legend(fontsize=7, frameon=False, ncol=2)
    axes[1].plot(tt, activity, color="0.2", lw=1)
    for cls in cls_colors:
        m = labels == cls
        axes[1].fill_between(tt, 0, activity.max(), where=m,
                             color=cls_colors[cls], alpha=0.18, label=cls)
    axes[1].axhline(args.static_thresh, color="0.5", ls=":", lw=1)
    axes[1].axhline(args.active_thresh, color="0.5", ls=":", lw=1)
    axes[1].set_ylabel("activity RMS [rad/s]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=8, frameon=False, ncol=3)
    fig.suptitle("dq timeline and motion segmentation")
    fig.tight_layout()
    fig.savefig(outdir / "timeline_segments.png", dpi=150)
    plt.close(fig)

    # -- α 权衡图（两个面板，同一 x 轴——不用双轴）
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    als = [r["alpha"] for r in alpha_rows]
    axes[0].plot(als, [r["noise_rms_ratio"] for r in alpha_rows], "o-", color="#4269d0")
    axes[0].set_xlabel("EMA alpha")
    axes[0].set_ylabel("residual noise RMS ratio (vs raw)")
    axes[0].grid(alpha=0.25, lw=0.5)
    axes[1].plot(als, [r["phase_lag_deg_at_fosc"] for r in alpha_rows], "o-", color="#ff725c")
    axes[1].set_xlabel("EMA alpha")
    axes[1].set_ylabel(f"phase lag @ {args.fosc} Hz [deg]")
    axes[1].grid(alpha=0.25, lw=0.5)
    fig.suptitle("EMA filter trade-off: noise reduction vs phase lag")
    fig.tight_layout()
    fig.savefig(outdir / "alpha_tradeoff.png", dpi=150)
    plt.close(fig)

    # -- dq vs 有限差分 放大对比（取最活跃 20s）
    center = int(np.argmax(np.convolve(activity, np.ones(int(20 * fs)) / (20 * fs), "same")))
    lo, hi = max(0, center - int(10 * fs)), min(len(tu), center + int(10 * fs))
    fig, ax = plt.subplots(figsize=(11, 4))
    jshow = int(np.argmax([s for s in (dqu[lo:hi] ** 2).mean(axis=0)]))
    ax.plot(tt[lo:hi], dq_fd[lo:hi, jshow], lw=0.7, color="#efb118",
            label="finite-diff of q")
    ax.plot(tt[lo:hi], dqu[lo:hi, jshow], lw=0.9, color="#4269d0",
            label="reported dq")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(f"{joint_names[jshow]} velocity [rad/s]")
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25, lw=0.5)
    fig.tight_layout()
    fig.savefig(outdir / "dq_vs_finite_diff.png", dpi=150)
    plt.close(fig)

    summary = {
        "npz": str(args.npz),
        "bag": str(data["bag"]) if "bag" in data else "",
        "fs_hz": fs,
        "jitter": jitter,
        "noise_split_hz": args.noise_split_hz,
        "segments": seg_stats,
        "alpha_sweep": alpha_rows,
    }
    with open(outdir / "summary.json", "w") as fh:
        json.dump(summary, fh, indent=2)

    print(json.dumps(summary, indent=2))
    print(f"\nplots + summary.json written to {outdir}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    pe = sub.add_parser("extract", help="rosbag -> npz（系统 python）")
    pe.add_argument("--bag", required=True)
    pe.add_argument("--out", required=True)
    pe.set_defaults(func=cmd_extract)

    pa = sub.add_parser("analyze", help="npz -> 频谱/定标报告（conda python）")
    pa.add_argument("--npz", required=True)
    pa.add_argument("--outdir", required=True)
    pa.add_argument("--fosc", type=float, default=1.86, help="震荡频率 Hz")
    pa.add_argument("--noise-split-hz", type=float, default=5.0,
                    help="信号/噪声分界（零相位低通截止）")
    pa.add_argument("--seg-window", type=float, default=1.0, help="分段滚动窗 s")
    pa.add_argument("--static-thresh", type=float, default=0.05,
                    help="static 段活动度阈值 rad/s")
    pa.add_argument("--active-thresh", type=float, default=0.3,
                    help="active 段活动度阈值 rad/s")
    pa.set_defaults(func=cmd_analyze)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
