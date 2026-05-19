"""End-of-path analysis: how far does the chassis overshoot the goal?"""
import struct, sys, bisect, math
from wpiutil.log import DataLogReader

log = sys.argv[1] if len(sys.argv) > 1 else max(__import__("glob").glob("logs/akit_*.wpilog"), key=__import__("os").path.getmtime)
print(f"Decoding {log}\n")
r = DataLogReader(log)

wanted = {
    "/RealOutputs/Drive/Sim/GroundTruthPose": "gt",
    "/RealOutputs/Drive/Pose": "pose",
    "/RealOutputs/PathPlanner/TargetPose": "tgt",
    "/RealOutputs/Drive/Speeds": "actSpeeds",
    "/RealOutputs/Drive/SetpointSpeeds": "setSpeeds",
    "/RealOutputs/PathPlanner/Diag/TargetLinearVel": "tgtV",
    "/RealOutputs/PathPlanner/LastResult": "lastResult",
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
        elif k in ("actSpeeds","setSpeeds") and len(d) >= 24:
            streams[k].append((ts, struct.unpack("<ddd", d[:24])))
        elif k == "tgtV" and len(d) >= 8:
            streams[k].append((ts, struct.unpack("<d", d[:8])[0]))
        elif k == "lastResult":
            try: streams[k].append((ts, d.decode("utf-8")))
            except: pass

t0 = streams["tgt"][0][0]
goal_pose = streams["tgt"][-1][1]  # last target pose
print(f"Path goal: ({goal_pose[0]:.3f}, {goal_pose[1]:.3f}, {math.degrees(goal_pose[2]):.1f}°)")
if streams["lastResult"]:
    print(f"LastResult: {streams['lastResult'][-1][1]!r} @ t={streams['lastResult'][-1][0]-t0:.2f}s")

def near(lst, t):
    if not lst: return None
    i = bisect.bisect_left([x[0] for x in lst], t)
    if i == len(lst): i -= 1
    if i > 0 and abs(lst[i-1][0]-t) < abs(lst[i][0]-t): i -= 1
    return lst[i] if abs(lst[i][0]-t) < 0.05 else None

# Print fine-grained data around end + settle
end_t = streams["tgt"][-1][0]
gt_at_end_path = near(streams["gt"], end_t)
pose_at_end_path = near(streams["pose"], end_t)
print(f"\nAt end of path target series (t={end_t-t0:.2f}s):")
print(f"  GT pose:    ({gt_at_end_path[1][0]:.3f}, {gt_at_end_path[1][1]:.3f})  dist from goal: {math.hypot(gt_at_end_path[1][0]-goal_pose[0], gt_at_end_path[1][1]-goal_pose[1])*1000:.0f}mm")
print(f"  Est pose:   ({pose_at_end_path[1][0]:.3f}, {pose_at_end_path[1][1]:.3f})  dist from goal: {math.hypot(pose_at_end_path[1][0]-goal_pose[0], pose_at_end_path[1][1]-goal_pose[1])*1000:.0f}mm")

# Find max overshoot in x (path goes +x)
max_x_gt = max(streams["gt"], key=lambda r: r[1][0])
print(f"\nMax x reached (GT): {max_x_gt[1][0]:.3f} @ t={max_x_gt[0]-t0:.2f}s  (overshoot past goal {goal_pose[0]:.3f}: {(max_x_gt[1][0]-goal_pose[0])*1000:.0f}mm)")
max_x_est = max(streams["pose"], key=lambda r: r[1][0])
print(f"Max x reached (Est): {max_x_est[1][0]:.3f} @ t={max_x_est[0]-t0:.2f}s  (overshoot past goal: {(max_x_est[1][0]-goal_pose[0])*1000:.0f}mm)")

# Final settled pose (300ms after path end)
settle_t = end_t + 0.3
gt_final = near(streams["gt"], settle_t)
if gt_final:
    print(f"\n300ms after path end (t={settle_t-t0:.2f}s):")
    print(f"  GT pose:  ({gt_final[1][0]:.3f}, {gt_final[1][1]:.3f})  dist from goal: {math.hypot(gt_final[1][0]-goal_pose[0], gt_final[1][1]-goal_pose[1])*1000:.0f}mm")

# Fine-grained near end
print(f"\n{'t':>5s} {'tgtX':>6s} {'tgtY':>6s}  {'gtX':>6s} {'gtY':>6s}  {'gtErr':>6s} {'tgt_v':>5s} {'act_v':>5s}")
sample_ts = sorted({round((t-t0)/0.04)*0.04+t0 for t,_ in streams["tgt"]+streams["gt"][:len(streams["tgt"])+30]})
for tau in sample_ts:
    if tau-t0 < 0.7: continue
    if tau-t0 > 2.0: break
    tgt = near(streams["tgt"], tau)
    gt = near(streams["gt"], tau)
    tv = near(streams.get("tgtV",[]), tau)
    aS = near(streams.get("actSpeeds",[]), tau)
    if not (tgt and gt): continue
    act_v = math.hypot(aS[1][0], aS[1][1]) if aS else 0
    err = math.hypot(gt[1][0]-tgt[1][0], gt[1][1]-tgt[1][1])
    print(f"{tau-t0:5.2f} {tgt[1][0]:6.3f} {tgt[1][1]:6.3f}  {gt[1][0]:6.3f} {gt[1][1]:6.3f}  {err*1000:5.0f}mm {(tv[1] if tv else 0):5.2f} {act_v:5.2f}")
