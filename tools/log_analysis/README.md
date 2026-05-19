# Log analysis scripts

Small Python scripts that decode AdvantageKit `.wpilog` files and print summaries of drive/path behavior. All read the latest `logs/akit_*.wpilog` by default; pass a log path as the first arg to override.

## Setup

These scripts require WPILib's `wpiutil` Python wheel. From the WPILib install:

```powershell
pip install ~/wpilib/2026/python-wheels/wpiutil-*.whl
```

## Scripts

| Script | What it shows |
|---|---|
| `diag.py` | Full drive diagnostic snapshot — speeds, targets, ground truth, headings. Spot oscillation, drift, rotation glitches. |
| `gt_vs_target.py` | Ground-truth pose vs PathPlanner target pose over time, plus chassis speed magnitude. |
| `modules.py` | Per-module commanded vs measured wheel speeds vs applied volts. Reveals saturation, slip, tracking failures. |
| `obstacles.py` | Decode `World/Obstacles/Rectangles` from a log and print each AABB. |
| `settle.py` | End-of-path overshoot analysis — how far does the chassis stop past the goal? |
| `split_paths.py` | Split a combined-auto log into per-path windows by detecting gaps in the target series. |
| `tracking_metrics.py` | Mean / max / RMS tracking error between target and ground-truth pose. |

## Usage

```powershell
# Run against the latest log
python tools/log_analysis/diag.py

# Run against a specific log
python tools/log_analysis/diag.py logs/akit_26-05-18_03-42-51.wpilog
```
