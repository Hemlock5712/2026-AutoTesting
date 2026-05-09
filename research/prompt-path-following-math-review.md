# Path Following Math & Logic Verification

## Context

This is an FRC swerve drive robot with two distance-based path following controllers that consume a `FollowablePath` interface:

1. **FollowPath** (`commands/FollowPath.java`) — PD controller with adaptive lookahead, cross-track correction, curvature feedforward. Production-proven.
2. **LQRFollower** (`commands/LQRFollower.java`) — LQR prototype using DARE solver, double-integrator model. New, needs verification.

Both consume `ArcLengthTrajectory` (`utils/path/ArcLengthTrajectory.java`) which re-parameterizes Choreo time-based trajectories into arc-length (distance-based). The robot tracks its position on the path, not a clock.

Physical limits are enforced by `AccelerationLimiter` (`commands/AccelerationLimiter.java`) which uses Kraken X60 FOC motor dyno data and friction circle constraints.

## Your Task

Read all four files completely, then verify the following:

### 1. ArcLengthTrajectory — Re-parameterization Correctness
- **Trapezoidal integration**: Verify the t→s integration in the constructor is mathematically correct. Check for off-by-one errors in the loop. Verify `sTable[0] = 0` and `sTable[n-1] = totalLength`.
- **Binary search + interpolation**: Verify `bracketIndex()` correctly finds `sTable[idx] <= s < sTable[idx+1]`. Check edge cases: s=0, s=totalLength, s slightly beyond bounds.
- **Tangent computation**: The tangent at arc-length s should be a unit vector. Verify the velocity-based tangent is normalized correctly. Check the fallback (finite difference) when velocity is near zero.
- **Curvature formula**: Verify the signed curvature `κ = (vx*ay - vy*ax) / |v|³` derivation. The conversion from time-based derivatives to arc-length-based derivatives has a speed factor — verify this is applied correctly.
- **Heading interpolation**: Verify shortest-angle interpolation wraps correctly (no discontinuity at ±π).
- **Projection (getClosestPointInRange)**: Verify the coarse search + Newton-Raphson refinement. Check: Is the Newton step `f(s) = dot(pathPoint - robotPos, tangent)` correct for finding the perpendicular foot? Is `f'(s) = 1 + κ * dot(diff, normal)` the correct derivative? Does it handle concave/convex regions?
- **getArcLengthAtTimestamp**: Verify the binary search on tSamples is correct and handles edge cases.

### 2. LQRFollower — Control Theory Correctness
- **State-space model**: The state is [crossTrackError, crossTrackRate, headingError, headingRate], input is [lateralAccel, angularAccel]. Verify the A and B matrices represent a correct double-integrator:
  - A should have 1s at (0,1) and (2,3), zeros elsewhere
  - B should have 1s at (1,0) and (3,1), zeros elsewhere
- **Discretization**: Verify `Discretization.discretizeAB(A, B, dt)` is called correctly and produces the expected discrete-time system.
- **DARE solver**: Verify the iterative DARE solver (`solveDAREIterative`) implements the correct equation: `P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q`. Check convergence criteria. Check if 50 iterations is sufficient.
- **Gain computation**: Verify `K = (R + B^T P B)^{-1} B^T P A` and `u = -K * x` are correct.
- **Bryson's Rule defaults**: Verify the default Q/R values match the claimed tolerances in comments (e.g., qCrossTrack = 400 should equal 1/(0.05)²).
- **withTolerances()**: Verify the rate tolerances (10x position/heading) are reasonable. Verify R computation from friction limit and drive base radius.
- **Velocity composition**: After computing `u = [lateralAccel, angularAccel]`:
  - Along-track: `tangent * profiledSpeed` — correct?
  - Lateral correction: `normal * lateralAccel * dt` — is integrating acceleration to get velocity correction correct here? Should this accumulate across cycles?
  - Omega: `lastOmega + angularAccel * dt` — same question about accumulation.
- **Cross-track error sign convention**: Verify positive crossTrackError = robot is LEFT of path (consistent with the signed cross product in ProjectionResult).
- **Heading error**: Verify `angleModulus(target - actual)` gives the correct sign for the LQR to correct in the right direction.
- **Cross-track rate via finite difference**: `(crossTrackError - lastCrossTrackError) / dt` — is this numerically stable? Would a filter help? Is this the right derivative (should it account for the robot's actual lateral velocity instead)?

### 3. FollowPath — PD Controller Review
- **Adaptive lookahead**: Verify `lookaheadK * speed + min` clamped to [min, max] makes physical sense. Does the curvature cap `lookaheadMaxArcAngle / kappa` prevent chord error on tight turns?
- **Cross-track PD**: Verify the signs. `correction = kp * error + kd * rate` applied as `nx * (-correction + curvatureFf)` — does negative correction push the robot back toward the path?
- **Curvature feedforward**: `curvatureFfGain * v² * κ` — this should be a lateral velocity term anticipating the path's curvature. Verify the gain units (seconds) and sign convention.
- **Rotation budget**: Verify the friction circle allocation (`sqrt(1 - angular²/max²)`) correctly reduces translation when rotation consumes friction budget.
- **angleErrorToOmega**: Verify the stopping-profile omega (`sqrt(2 * decel * error)`) prevents overshoot. Verify the three-way min (stopping, linear, budget) makes sense.
- **isFinished()**: Verify completion logic handles both stopping (endVelocity=0) and through-running (endVelocity>0) paths correctly.

### 4. Cross-Controller Consistency
- Both controllers use `PROJECTION_MAX_DELTA = 0.5` — is this the right value for both?
- Both feed through `AccelerationLimiter.integrateVelocity` — verify both call it with the same signature semantics.
- Do both handle the first-frame case (dt ≈ 0) correctly?
- Do both log to non-conflicting namespaces?

### 5. Potential Issues to Flag
- Any division by zero risks not guarded
- Any angle wrapping bugs
- Any cases where the LQR could produce infinite or NaN outputs
- Any numerical stability concerns with the DARE solver
- Any sign convention mismatches between the two controllers

## Rules
- Read all relevant files completely before making any claims
- Show your math when verifying formulas — don't just say "looks correct"
- If you find a bug, fix it and explain why
- If something is technically correct but could be more robust, note it as a suggestion
- Run `./gradlew build` after any changes
