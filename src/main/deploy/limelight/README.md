# Parade Limelight setup

Robot code selects pipeline slot **2** on every Limelight while
`ParadeConfig.ENABLED` is `true`. Competition pipeline slot **0** is left unchanged.

For each Limelight (`br`, `bl`, `fl`, `fr`, and `mm`):

1. Open the camera web interface while connected to the robot network.
2. Download/export pipeline 0 as a backup.
3. Duplicate pipeline 0 into slot 2 and name it `Parade`.
4. In slot 1's AprilTag 3D/field-localization settings, upload
   `parade-tags-3-4.fmap` from this directory.
5. Confirm the camera reports pipeline 2 and that its field-space pose changes heading when the
   trailer is rotated.

The parade map intentionally contains only tags 3 and 4 mounted to the modified Hub. They use the
standard 2026 welded-field X/Y positions and orientations of tags 9 and 10, respectively. Their
current Z position is provisional: `0.51435 m`, which is the standard `1.12395 m` height minus the
estimated 24-inch (`0.6096 m`) parade offset. Replace the Z value at index 11 of each tag's
`transform` after measuring the final installation.

`FieldInfo.HUB_HEIGHT` is lowered by 24 inches for geometry and simulation. The real shooter uses
empirical flywheel/hood lookup tables, so do not shoot at the modified Hub until a parade-specific
lookup has been measured and verified.

Before returning to competition, set `ParadeConfig.ENABLED` to `false`. That restores pipeline 0,
competition motion limits, MegaTag2 single-tag behavior, normal heading covariance, and normal Hub
height.
