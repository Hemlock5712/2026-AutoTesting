#!/usr/bin/env python3
"""Compare final Drive/Pose entries between two WPILOG files.

Used to verify that DriveReplay (Java pose estimator on logged 250 Hz inputs) lands
in approximately the same place as DriveCTRE (CTRE JNI estimator) did during sim.
Or to check how a vision std-dev change shifts the replayed pose.

Requires `robotpy-wpiutil` (already on the WPILib install path; otherwise pip install).
"""

import math
import struct
import sys
from pathlib import Path

from wpiutil.log import DataLogReader


def last_pose(path: Path, keys_priority: list[str]):
    """Returns (entry_name, last_timestamp_us, (x, y, theta_rad)) for the highest-priority
    `struct:Pose2d` key with `endswith` match against keys_priority."""
    reader = DataLogReader(str(path))
    entries: dict[int, tuple[str, str]] = {}
    for r in reader:
        if r.isStart():
            d = r.getStartData()
            entries[d.entry] = (d.name, d.type)

    last: dict[str, tuple[int, bytes]] = {}
    reader2 = DataLogReader(str(path))
    for r in reader2:
        if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
            continue
        info = entries.get(r.getEntry())
        if info is None:
            continue
        name, typ = info
        if typ != "struct:Pose2d":
            continue
        last[name] = (r.getTimestamp(), bytes(r.getRaw()))

    for key in keys_priority:
        for name, (ts, payload) in last.items():
            if name == key or name.endswith(key):
                if len(payload) < 24:
                    continue
                x, y, theta = struct.unpack_from("<ddd", payload, 0)
                return name, ts, (x, y, theta)
    return None


def main(argv):
    if len(argv) != 3:
        print(f"usage: {argv[0]} <sim_log.wpilog> <replay_log.wpilog>", file=sys.stderr)
        return 2

    sim_path = Path(argv[1])
    rep_path = Path(argv[2])

    sim_priority = [
        "/RealOutputs/Drive/Pose",
        "/Drive/Pose",
    ]
    rep_priority = [
        "/ReplayOutputs/Drive/Pose",
        "/Drive/Pose",
    ]

    sim = last_pose(sim_path, sim_priority)
    rep = last_pose(rep_path, rep_priority)

    if sim is None:
        print(f"{sim_path}: no struct:Pose2d under /RealOutputs/Drive/Pose")
        return 1
    if rep is None:
        print(f"{rep_path}: no struct:Pose2d under /ReplayOutputs/Drive/Pose")
        return 1

    sk, sts, (sx, sy, st) = sim
    rk, rts, (rx, ry, rt) = rep

    print(f"Sim    [{sk}] last @ {sts/1e6:.3f}s")
    print(f"  pose: x={sx:+.4f}  y={sy:+.4f}  theta={st:+.4f}rad ({math.degrees(st):+.2f}deg)")
    print(f"Replay [{rk}] last @ {rts/1e6:.3f}s")
    print(f"  pose: x={rx:+.4f}  y={ry:+.4f}  theta={rt:+.4f}rad ({math.degrees(rt):+.2f}deg)")

    dx = rx - sx
    dy = ry - sy
    dt = rt - st
    while dt > math.pi:
        dt -= 2 * math.pi
    while dt < -math.pi:
        dt += 2 * math.pi
    print(
        f"Delta: dx={dx:+.4f}m  dy={dy:+.4f}m  dtheta={dt:+.4f}rad ({math.degrees(dt):+.2f}deg)"
    )
    print(f"Translation error: {math.hypot(dx, dy):.4f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
