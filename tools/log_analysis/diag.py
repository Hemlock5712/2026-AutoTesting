"""
Full diagnostic decode for the latest auto run. Pulls speeds, target, ground truth, and headings
into one snapshot to spot oscillation, drift, and rotation glitches.
"""

import math
import struct
import bisect
import glob
import os
import sys

from wpiutil.log import DataLogReader


def pick_log():
    if len(sys.argv) > 1:
        return sys.argv[1]
    candidates = glob.glob("c:/Users/joeoj/Downloads/2026-AutoTesting/logs/akit_*.wpilog")
    return max(candidates, key=os.path.getmtime)


def decode_pose(d):
    return struct.unpack("<ddd", d[:24]) if len(d) >= 24 else None


def decode_double(d):
    return struct.unpack("<d", d[:8])[0] if len(d) >= 8 else None


WANTED = {
    "/RealOutputs/Drive/Pose": "pose",
    "/RealOutputs/Drive/Sim/GroundTruthPose": "gt",
    "/RealOutputs/PathPlanner/TargetPose": "target",
    "/RealOutputs/PathPlanner/Diag/TargetLinearVel": "tgtV",
    "/RealOutputs/PathPlanner/Diag/TargetHeadingRad": "tgtH",
    "/RealOutputs/Drive/SetpointSpeeds": "setSpeeds",
    "/RealOutputs/Drive/Speeds": "actSpeeds",
    "/RealOutputs/Drive/TranslationSpeedMps": "actV",
    "/RealOutputs/PathPlanner/LastResult": "lastResult",
}


def main():
    log = pick_log()
    print(f"Decoding {log}")
    print(f"Size: {os.path.getsize(log)/1e6:.1f} MB\n")

    r = DataLogReader(log)
    ids = {}
    for rec in r:
        if rec.isStart():
            sd = rec.getStartData()
            if sd.name in WANTED:
                ids[WANTED[sd.name]] = sd.entry

    print("Available entries:", sorted(ids.keys()))

    streams = {k: [] for k in ids}
    r2 = DataLogReader(log)
    for rec in r2:
        if rec.isStart():
            continue
        e = rec.getEntry()
        ts = rec.getTimestamp() / 1e6
        d = rec.getRaw()
        for k, eid in ids.items():
            if e != eid:
                continue
            if k in ("pose", "gt", "target") and len(d) >= 24:
                streams[k].append((ts, struct.unpack("<ddd", d[:24])))
            elif k in ("setSpeeds", "actSpeeds") and len(d) >= 24:
                streams[k].append((ts, struct.unpack("<ddd", d[:24])))
            elif k == "lastResult":
                try:
                    streams[k].append((ts, d.decode("utf-8")))
                except UnicodeDecodeError:
                    pass
            elif len(d) >= 8:
                streams[k].append((ts, struct.unpack("<d", d[:8])[0]))

    if not streams.get("target"):
        print("No TargetPose samples — auto never started.")
        return

    target = streams["target"]
    t0 = target[0][0]

    last_result = streams.get("lastResult") or []
    if last_result:
        ts, val = last_result[-1]
        print(f"LastResult: {val!r} @ t={ts - t0:.2f}s")
    else:
        print("LastResult: not emitted (auto still hung or interrupted)")
    print(f"Auto duration (target span): {target[-1][0] - t0:.2f}s  with {len(target)} target samples")

    def near(lst, t):
        if not lst:
            return None
        ts_list = [x[0] for x in lst]
        i = bisect.bisect_left(ts_list, t)
        if i == len(ts_list):
            i -= 1
        if i > 0 and abs(ts_list[i - 1] - t) < abs(ts_list[i] - t):
            i -= 1
        return lst[i] if abs(ts_list[i] - t) < 0.05 else None

    err_est = []
    err_gt = []
    drift = []
    heading_err_at_target = []
    for ts, tp in target:
        if ts < t0:
            continue
        p = near(streams.get("pose", []), ts)
        g = near(streams.get("gt", []), ts)
        if p:
            err_est.append((ts - t0, math.hypot(p[1][0] - tp[0], p[1][1] - tp[1])))
        if g:
            err_gt.append((ts - t0, math.hypot(g[1][0] - tp[0], g[1][1]-tp[1])))
            heading_err = (g[1][2] - tp[2] + math.pi) % (2 * math.pi) - math.pi
            heading_err_at_target.append((ts - t0, heading_err))
        if p and g:
            drift.append((ts - t0, math.hypot(p[1][0] - g[1][0], p[1][1] - g[1][1])))

    def stats(name, samples):
        if not samples:
            print(f"{name:30s} (no samples)")
            return
        xs = sorted(s[1] for s in samples)
        n = len(xs)
        mean = sum(xs) / n
        p95 = xs[int(n * 0.95)]
        mx = xs[-1]
        worst = max(samples, key=lambda s: s[1])
        print(
            f"{name:30s} mean={mean*1000:6.1f}mm  p95={p95*1000:6.1f}mm  max={mx*1000:6.1f}mm  worst@t={worst[0]:.2f}s"
        )

    print()
    stats("Drive/Pose vs Target", err_est)
    stats("GroundTruth vs Target", err_gt)
    stats("Pose drift (est vs GT)", drift)
    if heading_err_at_target:
        xs = sorted(abs(h[1]) for h in heading_err_at_target)
        n = len(xs)
        print(
            f"{'Heading err (GT vs target)':30s} mean={math.degrees(sum(xs)/n):6.1f}deg p95={math.degrees(xs[int(n*0.95)]):6.1f}deg max={math.degrees(xs[-1]):6.1f}deg"
        )

    # Velocity profile with target/setpoint/actual side by side
    print()
    print(f"{'t_auto':>7s} {'tgt_v':>7s} {'set_v':>7s} {'act_v':>7s} {'gt_err':>8s}")
    tgt_v = streams.get("tgtV", [])
    set_sp = streams.get("setSpeeds", [])
    act_v = streams.get("actV", [])
    gt_err_idx = {e[0]: e[1] for e in err_gt}
    sample_ts = sorted(set(round((t - t0) / 0.05) * 0.05 + t0 for t, _ in target if t >= t0))
    for t in sample_ts[: int(min(len(sample_ts), 50))]:
        tv = near(tgt_v, t)
        ssp = near(set_sp, t)
        av = near(act_v, t)
        if tv is None or ssp is None or av is None:
            continue
        sset = math.hypot(ssp[1][0], ssp[1][1])
        ge = gt_err_idx.get(t - t0, None)
        ge_s = f"{ge*1000:7.1f}" if ge is not None else "    -- "
        print(f"{t-t0:7.2f} {tv[1]:7.2f} {sset:7.2f} {av[1]:7.2f} {ge_s}mm")


if __name__ == "__main__":
    main()
