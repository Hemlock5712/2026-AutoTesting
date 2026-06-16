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

## Calibrating to the robot: deploy -> tune -> finalize

There are two kinds of constants:

**Measured once (compile-time constants in `ShotPhysics.java`)** — set, commit, deploy:
`LAUNCH_HEIGHT_M`, `HOOD_ZERO_ELEVATION_DEG` (+ the 1:1 slope), `WHEEL_RADIUS_M`,
`BALL_MASS_KG`, `BALL_DIAMETER_M`, `GOAL_HEIGHT_M`, `GOAL_OPENING_RADIUS_M`, and the
limits `ShotSolver.HOOD_MIN/MAX_DEG`, `FW_MIN/MAX_RPS`.

**Live tuning knobs (NetworkTables, no rebuild):**
- `Shooter/SlipEfficiency` — exit-speed-per-RPS. The make-window knob: spin up at a
  distance and sweep this until shots drop centered. It only rescales flywheel RPS
  (hood and ToF are slip-independent), so one value corrects every distance, live.
- `SWM/TiltComp` — 1 = tilt compensation on, 0 = off (verify the Pigeon sign first).

**The loop:**
1. **Deploy** once with the measured constants.
2. **Tune** `Shooter/SlipEfficiency` live until centered (covers your ball/compression).
   If shots are good near but off far (or vice-versa), that's the Magnus *shape* — adjust
   `SPIN_FACTOR` / `LIFT_COEFF_PER_SPIN` in `ShotPhysics`, regenerate the map, re-bake,
   `python3 verify.py`, redeploy. (Rare — only when the ball's backspin behavior changes.)
3. **Finalize**: copy the dialed-in `SlipEfficiency` into `ShotPhysics.SLIP_EFFICIENCY`
   and commit, so it's the default next boot.

The dual-wheel shooter (different diameters/gearing, reduced backspin) needs no structural
change: the exit-speed blend is absorbed by `SlipEfficiency`, and the reduced backspin by
the Magnus terms in step 2.

