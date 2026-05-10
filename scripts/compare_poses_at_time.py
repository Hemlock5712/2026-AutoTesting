#!/usr/bin/env python3
"""Compare /RealOutputs/Drive/Pose vs /ReplayOutputs/Drive/Pose at matching timestamps.

Walks both logs in parallel, samples the most-recent pose <= target_timestamp from each,
and reports the delta. Useful when AKit dedup makes the "last pose" comparison misleading
(e.g. one log keeps logging while the robot is stopped because vision is still updating it).

Usage:
  compare_poses_at_time.py SIM.wpilog REPLAY.wpilog [target_seconds]
      Sample at 25/50/75/100% of [0, target_seconds] (or last common timestamp if omitted).

  compare_poses_at_time.py SIM.wpilog REPLAY.wpilog --stride 1.0
      Sample every 1.0 s from the first common pose to the last common pose. Lines where
      both poses haven't moved since the previous row are collapsed (so a long parked tail
      doesn't drown out the motion phase).
"""

import math
import struct
import sys
from pathlib import Path

from wpiutil.log import DataLogReader

SIM_KEYS = ["/RealOutputs/Drive/Pose"]
REP_KEYS = ["/ReplayOutputs/Drive/Pose"]


def collect(path: Path, keys: list[str]):
    reader = DataLogReader(str(path))
    entries = {}
    for r in reader:
        if r.isStart():
            d = r.getStartData()
            entries[d.entry] = (d.name, d.type)

    samples = []  # (ts_seconds, x, y, theta)
    reader2 = DataLogReader(str(path))
    target_name = None
    for k in keys:
        for n, _ in entries.values():
            if n == k:
                target_name = k
                break
        if target_name:
            break

    if target_name is None:
        # Fuzzy fallback (suffix match on Drive/Pose, struct:Pose2d type)
        for n, t in entries.values():
            if t == "struct:Pose2d" and n.endswith("/Drive/Pose"):
                target_name = n
                break

    if target_name is None:
        return None, []

    for r in reader2:
        if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
            continue
        info = entries.get(r.getEntry())
        if info is None:
            continue
        name, typ = info
        if name != target_name or typ != "struct:Pose2d":
            continue
        payload = bytes(r.getRaw())
        if len(payload) < 24:
            continue
        x, y, theta = struct.unpack_from("<ddd", payload, 0)
        samples.append((r.getTimestamp() / 1e6, x, y, theta))
    samples.sort(key=lambda s: s[0])
    return target_name, samples


def sample_at(samples, target_ts):
    """Return the latest sample with ts <= target_ts, or None if no such sample."""
    last = None
    for s in samples:
        if s[0] > target_ts:
            break
        last = s
    return last


def _print_header():
    print(
        f"{'time(s)':>8} {'sim x':>9} {'sim y':>9} {'sim th':>8} | "
        f"{'rep x':>9} {'rep y':>9} {'rep th':>8} | {'dxy(m)':>8} {'moved?':>7}"
    )


def _print_row(t, s, r, prev_s, prev_r):
    dx, dy = r[1] - s[1], r[2] - s[2]
    derr = math.hypot(dx, dy)
    moved_sim = prev_s is None or abs(s[1] - prev_s[1]) + abs(s[2] - prev_s[2]) > 1e-4 or abs(s[3] - prev_s[3]) > 1e-4
    moved_rep = prev_r is None or abs(r[1] - prev_r[1]) + abs(r[2] - prev_r[2]) > 1e-4 or abs(r[3] - prev_r[3]) > 1e-4
    flag = "moving" if (moved_sim or moved_rep) else "parked"
    print(
        f"{t:>8.2f} {s[1]:>+9.4f} {s[2]:>+9.4f} {math.degrees(s[3]):>+8.2f} | "
        f"{r[1]:>+9.4f} {r[2]:>+9.4f} {math.degrees(r[3]):>+8.2f} | {derr:>8.6f} {flag:>7}"
    )


def main(argv):
    if len(argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2

    sim_path = Path(argv[1])
    rep_path = Path(argv[2])

    sim_key, sim_samples = collect(sim_path, SIM_KEYS)
    rep_key, rep_samples = collect(rep_path, REP_KEYS)

    if not sim_samples or not rep_samples:
        print("Missing pose samples", file=sys.stderr)
        return 1

    print(f"Sim    [{sim_key}]: {len(sim_samples)} samples in [{sim_samples[0][0]:.3f}, {sim_samples[-1][0]:.3f}]s")
    print(f"Replay [{rep_key}]: {len(rep_samples)} samples in [{rep_samples[0][0]:.3f}, {rep_samples[-1][0]:.3f}]s")
    print()

    stride = None
    target_ts = None
    extra = argv[3:]
    while extra:
        a = extra.pop(0)
        if a == "--stride":
            stride = float(extra.pop(0))
        else:
            target_ts = float(a)

    if stride is not None:
        # Walk from first to last common timestamp at fixed stride.
        t0 = max(sim_samples[0][0], rep_samples[0][0])
        t1 = min(sim_samples[-1][0], rep_samples[-1][0])
        print(f"Sampling every {stride:.2f}s from {t0:.2f}s to {t1:.2f}s:")
        _print_header()
        prev_s = prev_r = None
        max_dxy = 0.0
        moving_count = 0
        t = t0
        while t <= t1 + 1e-9:
            s = sample_at(sim_samples, t)
            r = sample_at(rep_samples, t)
            if s is not None and r is not None:
                _print_row(t, s, r, prev_s, prev_r)
                dx, dy = r[1] - s[1], r[2] - s[2]
                max_dxy = max(max_dxy, math.hypot(dx, dy))
                if prev_s is None or abs(s[1] - prev_s[1]) + abs(s[2] - prev_s[2]) > 1e-4:
                    moving_count += 1
                prev_s, prev_r = s, r
            t += stride
        print()
        print(f"max |dxy| over all samples: {max_dxy:.6f} m  (rows where sim moved: {moving_count})")
        return 0

    # Default: 25/50/75/100% of target.
    if target_ts is None:
        target_ts = min(sim_samples[-1][0], rep_samples[-1][0])
    print(f"Sample fractions of [0, {target_ts:.2f}]s:")
    _print_header()
    prev_s = prev_r = None
    for f in [0.25, 0.5, 0.75, 1.0]:
        t = target_ts * f
        s = sample_at(sim_samples, t)
        r = sample_at(rep_samples, t)
        if s is None or r is None:
            continue
        _print_row(t, s, r, prev_s, prev_r)
        prev_s, prev_r = s, r
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
