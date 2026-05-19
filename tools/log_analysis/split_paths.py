"""Split combined-auto log into per-path windows by detecting gaps in target series."""
import sys, struct, bisect, math
from wpiutil.log import DataLogReader

log = sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-05-18_22-53-53.wpilog"
r = DataLogReader(log)

wanted = {
    "/RealOutputs/Drive/Sim/GroundTruthPose": "gt",
    "/RealOutputs/Drive/Pose": "pose",
    "/RealOutputs/PathPlanner/TargetPose": "tgt",
    "/RealOutputs/PathPlanner/LastResult": "lastResult",
    "/RealOutputs/PathPlanner/PathName": "pathName",
    "/RealOutputs/PathPlanner/AutoName": "autoName",
}
ids = {}
for rec in r:
    if rec.isStart():
        sd = rec.getStartData()
        if sd.name in wanted: ids[wanted[sd.name]] = sd.entry

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
        elif k in ("lastResult","pathName","autoName"):
            try: streams[k].append((ts, d.decode("utf-8")))
            except: pass

# Use autoName transitions to find segment boundaries
target = streams["tgt"]
auto_events = sorted(streams.get("autoName", []))
boundaries = []
if auto_events:
    boundaries.append(auto_events[0][0])
    for i in range(1, len(auto_events)):
        boundaries.append(auto_events[i][0] - 0.05)  # tiny gap to separate
        boundaries.append(auto_events[i][0])
    boundaries.append(target[-1][0])
else:
    boundaries = [target[0][0], target[-1][0]]

print(f"Target time range: {target[0][0]:.2f} -> {target[-1][0]:.2f}  ({len(target)} samples)")
print(f"Path names logged: {[(t, n) for t, n in streams.get('pathName', [])[:10]]}")
print(f"Auto names logged: {[(t, n) for t, n in streams.get('autoName', [])[:10]]}")
print(f"LastResults logged: {[(t, r) for t, r in streams.get('lastResult', [])[:10]]}")
print(f"Segments detected: {len(boundaries)//2}")

def near(lst, t):
    if not lst: return None
    i = bisect.bisect_left([x[0] for x in lst], t)
    if i == len(lst): i -= 1
    if i > 0 and abs(lst[i-1][0]-t) < abs(lst[i][0]-t): i -= 1
    return lst[i] if abs(lst[i][0]-t) < 0.05 else None

def stats(name, errs):
    if not errs:
        print(f"  {name}: no samples")
        return
    xs = sorted(e[1] for e in errs)
    n = len(xs)
    mean = sum(xs)/n
    p95 = xs[int(n*0.95)]
    mx = xs[-1]
    worst = max(errs, key=lambda e: e[1])
    print(f"  {name:30s} n={n:3d} mean={mean*1000:6.1f}mm p95={p95*1000:6.1f}mm max={mx*1000:6.1f}mm worst@t={worst[0]:.2f}s")

# Per-segment stats
for i in range(0, len(boundaries), 2):
    seg_start = boundaries[i]
    seg_end = boundaries[i+1]
    seg_target = [(t, p) for t, p in target if seg_start <= t <= seg_end]
    t0 = seg_target[0][0]
    print(f"\n=== Segment {i//2 + 1}: t={seg_start:.2f}s -> {seg_end:.2f}s ({seg_end-seg_start:.2f}s, {len(seg_target)} samples) ===")
    err_est = []
    err_gt = []
    for ts, tp in seg_target:
        p = near(streams["pose"], ts)
        g = near(streams["gt"], ts)
        if p: err_est.append((ts-t0, math.hypot(p[1][0]-tp[0], p[1][1]-tp[1])))
        if g: err_gt.append((ts-t0, math.hypot(g[1][0]-tp[0], g[1][1]-tp[1])))
    stats("Drive/Pose vs Target", err_est)
    stats("GroundTruth vs Target", err_gt)
    # End position
    if g and seg_target:
        last_tp = seg_target[-1][1]
        last_g = near(streams["gt"], seg_target[-1][0])
        if last_g:
            print(f"  Final GT: ({last_g[1][0]:.3f}, {last_g[1][1]:.3f})  Target: ({last_tp[0]:.3f}, {last_tp[1]:.3f})  diff={math.hypot(last_g[1][0]-last_tp[0], last_g[1][1]-last_tp[1])*1000:.0f}mm")
