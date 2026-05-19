"""Decode and print all obstacles + check which ones intersect a candidate path."""
import struct, sys
from wpiutil.log import DataLogReader

log = sys.argv[1] if len(sys.argv) > 1 else "logs/akit_26-05-18_03-42-51.wpilog"
r = DataLogReader(log)
ids = {}
for rec in r:
    if rec.isStart():
        sd = rec.getStartData()
        if "Obstacles" in sd.name:
            ids[sd.name] = sd.entry

raw_records = {}
r2 = DataLogReader(log)
for rec in r2:
    if rec.isStart(): continue
    e = rec.getEntry()
    for name, eid in ids.items():
        if e == eid:
            raw_records.setdefault(name, []).append(rec.getRaw())

# Rectangle2d struct: per WPILib, it's (center_x, center_y, rotation_rad, xWidth, yWidth) — 5 doubles = 40 bytes
def rects(data):
    n = len(data) // 40
    out = []
    for i in range(n):
        cx, cy, rot, xw, yw = struct.unpack("<ddddd", data[i*40:(i+1)*40])
        out.append((cx, cy, rot, xw, yw))
    return out

for name, recs in raw_records.items():
    if not recs: continue
    d = recs[0]
    if "Rectangles" in name:
        rs = rects(d)
        print(f"\n{name}: {len(rs)} rects")
        for cx, cy, rot, xw, yw in rs:
            print(f"  center=({cx:.2f},{cy:.2f}) rot={rot:.2f} size=({xw:.2f}x{yw:.2f})  -> x=[{cx-xw/2:.2f},{cx+xw/2:.2f}] y=[{cy-yw/2:.2f},{cy+yw/2:.2f}]")
