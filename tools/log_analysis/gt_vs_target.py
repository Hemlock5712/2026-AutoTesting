"""GT pose vs target pose over time, plus chassis speed magnitude."""
import sys, struct, bisect, math
from wpiutil.log import DataLogReader

log = sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-05-18_03-42-51.wpilog"
r = DataLogReader(log)

wanted = {
    "/RealOutputs/Drive/Sim/GroundTruthPose": "gt",
    "/RealOutputs/Drive/Pose": "pose",
    "/RealOutputs/PathPlanner/TargetPose": "tgt",
    "/RealOutputs/Drive/Speeds": "actSpeeds",
    "/RealOutputs/PathPlanner/Diag/TargetLinearVel": "tgtV",
}
ids = {}
for rec in r:
    if rec.isStart():
        sd = rec.getStartData()
        if sd.name in wanted:
            ids[wanted[sd.name]] = sd.entry

streams = {k: [] for k in ids}
r2 = DataLogReader(log)
for rec in r2:
    if rec.isStart(): continue
    e = rec.getEntry()
    ts = rec.getTimestamp() / 1e6
    d = rec.getRaw()
    for k, eid in ids.items():
        if e != eid: continue
        if k in ("gt","pose","tgt") and len(d) >= 24:
            streams[k].append((ts, struct.unpack("<ddd", d[:24])))
        elif k == "actSpeeds" and len(d) >= 24:
            streams[k].append((ts, struct.unpack("<ddd", d[:24])))
        elif k == "tgtV" and len(d) >= 8:
            streams[k].append((ts, struct.unpack("<d", d[:8])[0]))

if not streams.get("tgt"):
    print("No tgt")
    sys.exit()

t0 = streams["tgt"][0][0]

def near(lst, t):
    if not lst: return None
    i = bisect.bisect_left([x[0] for x in lst], t)
    if i == len(lst): i -= 1
    if i > 0 and abs(lst[i-1][0]-t) < abs(lst[i][0]-t): i -= 1
    return lst[i] if abs(lst[i][0]-t) < 0.05 else None

print(f"{'t':>5s} {'gtX':>6s} {'gtY':>6s} {'gtH':>6s}  {'tgtX':>6s} {'tgtY':>6s}  {'poseX':>6s} {'poseY':>6s}  {'actVx':>6s} {'actVy':>6s} {'actW':>6s}  {'tgtV':>5s}")
sample_ts = sorted({round((t-t0)/0.05)*0.05+t0 for t,_ in streams["tgt"]})
for tau in sample_ts:
    if tau-t0 > 3.0: break
    gt = near(streams.get("gt",[]), tau)
    pose = near(streams.get("pose",[]), tau)
    tgt = near(streams.get("tgt",[]), tau)
    aS = near(streams.get("actSpeeds",[]), tau)
    tV = near(streams.get("tgtV",[]), tau)
    if not (gt and pose and tgt and aS): continue
    print(f"{tau-t0:5.2f} {gt[1][0]:6.3f} {gt[1][1]:6.3f} {math.degrees(gt[1][2]):6.1f}  "
          f"{tgt[1][0]:6.3f} {tgt[1][1]:6.3f}  "
          f"{pose[1][0]:6.3f} {pose[1][1]:6.3f}  "
          f"{aS[1][0]:6.2f} {aS[1][1]:6.2f} {aS[1][2]:6.2f}  "
          f"{(tV[1] if tV else 0):5.2f}")
