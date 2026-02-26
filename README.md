# Install `Robot_2026` in AdvantageScope

Use this quick process to install the robot model in AdvantageScope.

1. Open AdvantageScope.
2. In the menu bar, click `App` (macOS) or `AdvantageScope` (Windows/Linux), then select `Show Assets Folder`.
3. Download `Robot_2026` from Google Drive https://drive.google.com/drive/folders/1M_KRpmZVoDL1LgyMo935e72sWXf8dQNe?usp=sharing
4. Copy the entire `Robot_2026` folder into the AdvantageScope assets folder you opened in step 2.
5. Restart AdvantageScope (or reopen the 3D view) if needed, then select the robot in the visualization settings.

The order of the components for custom articulated parts is as follows:
- 0: Robot Chassis
- 1: Turret
- 2: Hood
- 3: Front 4 Bar Arm
- 4: Back 4 Bar Arm
- 5: Intake main structure

> Turret/Hood and the 3 turret components will be grouped together in the AdvantageScope components

## Notes

- Keep the folder name as `Robot_2026` and copy the full folder (including `config.json`, `model.glb`, and any `model_*.glb` files).
- This follows AdvantageScope's custom asset format, where each asset lives in its own subfolder inside the user assets folder.
- Alternative workflow from the docs: `Use Custom Assets Folder` can point AdvantageScope at a version-controlled assets parent folder.
# Test change for Claude workflow
