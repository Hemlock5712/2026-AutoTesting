# Offline shot-model verification (Python)

A faithful Python mirror of the Java shooter targeting stack
(`ShotPhysics` / `ShotSolver` / `ShotMapGenerator` / `ShooterMap` / `SwmTargeting`).
Because the WPILib Java can't be compiled everywhere, this lets us verify the
targeting logic and generate the model **offline, without the robot**.

## Usage

```
pip install numpy matplotlib
cd tools/shot_model

python3 verify.py    # cross-check vs the Java baked values + closed-loop end-to-end miss report
python3 plots.py     # write figures/ (trajectories, schedules, surfaces, end-to-end miss heatmap)
```

## What `verify.py` proves

1. **Cross-check** — re-solves every feasible grid node in Python and confirms it
   matches the values baked into `ShooterMap.java` (so the mirror is faithful).
2. **End-to-end** — for a sweep of robot states (moving, tilted), runs the
   `SwmTargeting` solve, then fires the resulting commands through an independent
   3D ball sim (gravity + drag + Magnus) with the inherited velocity and tilt, and
   reports the landing miss per category. Every feasible shot should land inside
   the hub opening. This is the same logic as the Java `SwmEndToEndTest`.

Keep the constants here in sync with `ShotPhysics.java`.
