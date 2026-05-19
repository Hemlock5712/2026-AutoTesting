"""
Tracking-error metrics for the SwerveSetpointGenerator migration A/B.

Decodes the latest akit_*.wpilog and reports:
  - Mean and max cross-track error (Drive/Pose vs PathPlanner/TargetPose)
  - Final position error after PathPlannerSmokeTest/LastResult == "finished"
  - Max |Drive/Pose - Drive/Sim/GroundTruthPose| (simulated slip)
  - Total sim wall-clock duration analyzed

Usage:
  python tmp_tracking_metrics.py [path_to_wpilog]

If no path is given, picks the latest logs/akit_*.wpilog by mtime.
"""

import glob
import math
import os
import struct
import sys

from wpiutil.log import DataLogReader


def pick_log():
    if len(sys.argv) > 1:
        return sys.argv[1]
    candidates = glob.glob("logs/akit_*.wpilog")
    if not candidates:
        raise SystemExit("No akit_*.wpilog files in logs/")
    return max(candidates, key=os.path.getmtime)


def decode_pose(data):
    # WPILib Pose2d struct: x (f64), y (f64), theta (f64)
    return struct.unpack("<ddd", data[:24]) if len(data) >= 24 else None


def decode_string(data):
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None


def main():
    log = pick_log()
    print(f"Decoding {log}")
    print(f"Size: {os.path.getsize(log)/1e6:.1f} MB\n")

    r = DataLogReader(log)
    entry_to_name = {}
    entry_to_type = {}
    for rec in r:
        if rec.isStart():
            sd = rec.getStartData()
            entry_to_name[sd.entry] = sd.name
            entry_to_type[sd.entry] = sd.type

    wanted = {
        "/RealOutputs/Drive/Pose": "drive_pose",
        "/RealOutputs/PathPlanner/TargetPose": "target_pose",
        "/RealOutputs/Drive/Sim/GroundTruthPose": "gt_pose",
        "/RealOutputs/PathPlannerSmokeTest/LastResult": "last_result",
        "/RealOutputs/PathPlannerSmokeTest/AutoName": "auto_name",
    }
    eids = {}
    for eid, name in entry_to_name.items():
        if name in wanted:
            eids[wanted[name]] = eid
    print(f"Entries found: {sorted(eids.keys())}")
    if "drive_pose" not in eids or "target_pose" not in eids:
        raise SystemExit("Missing required pose entries; sim probably didn't run an auto.")

    streams = {k: [] for k in eids}
    r2 = DataLogReader(log)
    for rec in r2:
        if rec.isStart():
            continue
        e = rec.getEntry()
        ts = rec.getTimestamp() / 1e6  # seconds
        for k, k_eid in eids.items():
            if e != k_eid:
                continue
            if k == "last_result":
                s = decode_string(rec.getRaw())
                if s is not None:
                    streams[k].append((ts, s))
            elif k == "auto_name":
                s = decode_string(rec.getRaw())
                if s is not None:
                    streams[k].append((ts, s))
            else:
                p = decode_pose(rec.getRaw())
                if p is not None:
                    streams[k].append((ts, p))

    print(f"\nSamples:")
    for k, v in streams.items():
        print(f"  {k}: {len(v)}")

    if "auto_name" in streams and streams["auto_name"]:
        print(f"\nAuto run: {streams['auto_name'][-1][1]}")
    if "last_result" in streams and streams["last_result"]:
        ts_done, result = streams["last_result"][-1]
        print(f"LastResult: {result!r} @ t={ts_done:.3f}s")
    else:
        ts_done = None

    drive = streams["drive_pose"]
    target = streams["target_pose"]

    # Pair each TargetPose sample with the nearest Drive/Pose by timestamp (binary search).
    drive_ts = [p[0] for p in drive]

    def nearest(stream_ts, t):
        # Binary search for nearest index
        lo, hi = 0, len(stream_ts) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if stream_ts[mid] < t:
                lo = mid + 1
            else:
                hi = mid
        # lo is the first >= t. Compare with lo-1.
        if lo > 0 and abs(stream_ts[lo - 1] - t) < abs(stream_ts[lo] - t):
            return lo - 1
        return lo

    errors = []
    t_first_target = target[0][0] if target else None
    for ts, tgt in target:
        if not drive:
            continue
        i = nearest(drive_ts, ts)
        ts_d, dp = drive[i]
        if abs(ts_d - ts) > 0.05:
            continue  # too stale, skip
        errors.append((ts, math.hypot(dp[0] - tgt[0], dp[1] - tgt[1])))

    if not errors:
        print("\nNo paired drive/target samples — auto probably never started.")
        return

    err_only = [e[1] for e in errors]
    mean_e = sum(err_only) / len(err_only)
    max_e = max(err_only)
    max_idx = err_only.index(max_e)
    max_t = errors[max_idx][0]
    p95 = sorted(err_only)[int(len(err_only) * 0.95)]
    print(f"\nCross-track error vs TargetPose:")
    print(f"  Samples: {len(errors)}")
    print(f"  Mean:    {mean_e*1000:7.1f} mm")
    print(f"  p95:     {p95*1000:7.1f} mm")
    print(f"  Max:     {max_e*1000:7.1f} mm  @ t={max_t:.2f}s")

    # Final position error: drive pose at LastResult timestamp vs target pose at same.
    if ts_done is not None:
        i_d = nearest(drive_ts, ts_done)
        # For target, walk backward to find last valid target sample before ts_done
        target_at_done = None
        for ts_t, p in reversed(target):
            if ts_t <= ts_done:
                target_at_done = (ts_t, p)
                break
        if target_at_done is not None:
            ts_t, tgt = target_at_done
            dp = drive[i_d][1]
            final_err = math.hypot(dp[0] - tgt[0], dp[1] - tgt[1])
            print(f"\nFinal-position error at LastResult: {final_err*1000:.1f} mm")
            print(f"  Drive  pose: ({dp[0]:.3f}, {dp[1]:.3f}, {math.degrees(dp[2]):.1f} deg)")
            print(f"  Target pose: ({tgt[0]:.3f}, {tgt[1]:.3f}, {math.degrees(tgt[2]):.1f} deg)")

    # Slip indicator (only meaningful if GroundTruthPose exists)
    if streams.get("gt_pose"):
        gt = streams["gt_pose"]
        gt_ts = [p[0] for p in gt]
        slips = []
        for ts, dp in drive:
            j = nearest(gt_ts, ts)
            if abs(gt[j][0] - ts) > 0.05:
                continue
            gp = gt[j][1]
            slips.append(math.hypot(dp[0] - gp[0], dp[1] - gp[1]))
        if slips:
            print(f"\n|Drive/Pose - Drive/Sim/GroundTruthPose|:")
            print(f"  Mean: {sum(slips)/len(slips)*1000:.1f} mm")
            print(f"  Max:  {max(slips)*1000:.1f} mm")


if __name__ == "__main__":
    main()
