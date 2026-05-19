"""
Per-module diagnostic: commanded module target speeds vs measured wheel speeds vs applied volts.
Reveals when sim wheels saturate, slip, or fail to track commanded velocity.
"""

import sys
import struct
import bisect
from wpiutil.log import DataLogReader


def main(log_path):
    print(f"Decoding {log_path}\n")
    r = DataLogReader(log_path)

    # Module-level + chassis-level entries
    wanted = {}
    for i in range(4):
        wanted[f"/Drive/Module{i}/DriveVelocityRadPerSec"] = f"m{i}_velRad"
        wanted[f"/Drive/Module{i}/DriveAppliedVolts"] = f"m{i}_volts"
        wanted[f"/Drive/Module{i}/DriveCurrentAmps"] = f"m{i}_amps"
        wanted[f"/Drive/Module{i}/TurnPosition"] = f"m{i}_turnPos"
    wanted["/RealOutputs/Drive/ModuleTargets"] = "modTargets"
    wanted["/RealOutputs/Drive/ModuleStates"] = "modStates"
    wanted["/RealOutputs/Drive/SetpointSpeeds"] = "setSpeeds"
    wanted["/RealOutputs/Drive/Speeds"] = "actSpeeds"
    wanted["/RealOutputs/PathPlanner/Diag/TargetLinearVel"] = "tgtV"
    wanted["/RealOutputs/Drive/Sim/PoseErrorMeters"] = "poseErr"

    ids = {}
    for rec in r:
        if rec.isStart():
            sd = rec.getStartData()
            if sd.name in wanted:
                ids[wanted[sd.name]] = sd.entry

    print("Available:", sorted(ids.keys()))

    streams = {k: [] for k in ids}
    r2 = DataLogReader(log_path)
    for rec in r2:
        if rec.isStart():
            continue
        e = rec.getEntry()
        ts = rec.getTimestamp() / 1e6
        d = rec.getRaw()
        for k, eid in ids.items():
            if e != eid:
                continue
            if k.startswith("mod") and "Targets" in (k + "Targets") and len(d) >= 96:
                # SwerveModuleState[] is 4 * (double speed, Rotation2d) -- but AKit logs may pack differently.
                # Try the AKit format: 4 modules of (speed, angle.radians) = 4 * 16 = 64 bytes
                vals = struct.unpack("<8d", d[:64])
                streams[k].append((ts, vals))  # [s0,a0,s1,a1,s2,a2,s3,a3]
            elif k in ("modTargets", "modStates") and len(d) >= 64:
                vals = struct.unpack("<8d", d[:64])
                streams[k].append((ts, vals))
            elif k in ("setSpeeds", "actSpeeds") and len(d) >= 24:
                streams[k].append((ts, struct.unpack("<ddd", d[:24])))
            elif k in ("tgtV", "poseErr") and len(d) >= 8:
                streams[k].append((ts, struct.unpack("<d", d[:8])[0]))
            elif k.startswith("m") and "_" in k and len(d) >= 8:
                streams[k].append((ts, struct.unpack("<d", d[:8])[0]))

    # Pick a reference time series for sampling
    ref = streams.get("modTargets") or streams.get("setSpeeds")
    if not ref:
        print("No ModuleTargets or SetpointSpeeds — auto never started.")
        return
    t0 = ref[0][0]

    def near(lst, t):
        if not lst:
            return None
        i = bisect.bisect_left([x[0] for x in lst], t)
        if i == len(lst):
            i -= 1
        if i > 0 and abs(lst[i - 1][0] - t) < abs(lst[i][0] - t):
            i -= 1
        return lst[i] if abs(lst[i][0] - t) < 0.05 else None

    WHEEL_RADIUS = 0.0508  # rough; from TunerConstants

    print()
    print(f"{'t':>5s} {'tgt_v':>5s} {'set_v':>5s} {'act_v':>5s}  "
          f"{'t0':>6s} {'m0':>6s} {'v0':>5s}  "
          f"{'t1':>6s} {'m1':>6s} {'v1':>5s}  "
          f"{'t2':>6s} {'m2':>6s} {'v2':>5s}  "
          f"{'t3':>6s} {'m3':>6s} {'v3':>5s}  "
          f"{'poseErr':>7s}")
    sample_ts = sorted({round((t - t0) / 0.05) * 0.05 + t0 for t, _ in ref})
    for tau in sample_ts:
        ts = tau
        if ts - t0 > 3.0:
            break
        modT = near(streams.get("modTargets", []), ts)
        modS = near(streams.get("modStates", []), ts)
        setSp = near(streams.get("setSpeeds", []), ts)
        actSp = near(streams.get("actSpeeds", []), ts)
        tgtV = near(streams.get("tgtV", []), ts)
        peErr = near(streams.get("poseErr", []), ts)
        if not (modT and modS and setSp and actSp):
            continue
        import math as m
        set_v = m.hypot(setSp[1][0], setSp[1][1])
        act_v = m.hypot(actSp[1][0], actSp[1][1])
        tgt_v_val = tgtV[1] if tgtV else 0.0
        pe_val = peErr[1] if peErr else 0.0

        # ModuleTargets is [speed0, angle0, speed1, angle1, ...]
        t_speeds = [modT[1][i*2] for i in range(4)]
        m_speeds = [modS[1][i*2] for i in range(4)]
        m_volts = []
        for i in range(4):
            v = near(streams.get(f"m{i}_volts", []), ts)
            m_volts.append(v[1] if v else 0.0)

        print(f"{ts-t0:5.2f} {tgt_v_val:5.2f} {set_v:5.2f} {act_v:5.2f}  "
              + "  ".join(f"{t_speeds[i]:6.2f} {m_speeds[i]:6.2f} {m_volts[i]:5.1f}" for i in range(4))
              + f"  {pe_val*1000:6.0f}mm")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-05-18_03-42-51.wpilog")
