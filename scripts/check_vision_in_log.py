#!/usr/bin/env python3
"""Check whether a wpilog contains any vision observations (hasObservation=true)."""
import sys
import struct
from wpiutil.log import DataLogReader

if len(sys.argv) != 2:
    print("usage: check_vision_in_log.py <log.wpilog>", file=sys.stderr)
    sys.exit(2)

reader = DataLogReader(sys.argv[1])
entries = {}
for r in reader:
    if r.isStart():
        d = r.getStartData()
        entries[d.entry] = (d.name, d.type)

reader2 = DataLogReader(sys.argv[1])
obs_keys = {}
all_vision_keys = set()
for r in reader2:
    if r.isStart() or r.isFinish() or r.isControl() or r.isSetMetadata():
        continue
    name, typ = entries.get(r.getEntry(), ("", ""))
    if "/Vision/" in name or "Vision" in name:
        all_vision_keys.add(name)
    if name.endswith("/HasObservation") or name.endswith("/hasObservation"):
        val = r.getRaw()[0] != 0
        obs_keys.setdefault(name, [0, 0])
        obs_keys[name][int(val)] += 1

print("Vision-related keys present:")
for k in sorted(all_vision_keys):
    print(f"  {k}")
print()
print("hasObservation true/false counts per camera:")
for k, (false_count, true_count) in sorted(obs_keys.items()):
    print(f"  {k}: true={true_count} false={false_count}")
