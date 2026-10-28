# MEBC-style test buoys for VRX

Simple 3D stand-ins for the real Monaco Energy Boat Challenge buoys, textured with crops of our
own MEBC 2026 photos, to test perception in the simulator.

| Model | Real object | Detected by `buoys_best.pt` in the sim (row at 10 m) |
| --- | --- | --- |
| `slalom_buoy.sdf` | white inflatable slalom pylon with the blue dot rosette | yes, 0.81 |
| `green_buoy.sdf` | green ring marker ("MARK SET BOT") on a blue base | yes, ~0.35–0.5 |
| `orange_float.sdf` | orange dock float | yes |
| `white_float.sdf` | white dock float | yes, ~0.4–0.5 |
| `mooring_buoy.sdf` | white mooring buoy with rusty post and shackle | not yet |

Sizes are estimates from photos. All models are static (they don't move with the waves).

## Run it (one command)

1. Start the sim and wait for `ScoringPlugin::OnRunning` in the log:
   `make vrx WORLD=navigation_task HEADLESS=True`
2. In a `make sh` terminal:
   `ros2 launch /ws/src/boat/sim_models/sim_buoy_test.launch.py`

This bridges the camera and LiDAR, places the five buoys in a row in front of the boat, starts
`yolo_detector` with our buoy model as `buoy_detector`, and opens a viewer on
`/perception/buoy_overlay`. Options are listed at the top of the launch file
(e.g. `at:=15 spacing:=5`).

`buoys_best.pt` is not in git (too big): get it from Jana and put it in `models/`.

## Known issue: `navigation_task` and the camera bridge

Inside `navigation_task.sdf` the world is named `sydney_regatta`, but VRX builds its bridge topics
from the file name, so no camera or LiDAR data reaches ROS. The launch file runs its own bridge
for the real name. For a world whose own bridge works (e.g. `perception_task`), pass `bridge:=false`.

## By hand

- Spawn/remove buoys: `python3 /ws/src/boat/sim_models/spawn_test_course.py --line` · `--remove` · `--help`
- Textures are in `textures/`; each `.sdf` says which ones it uses.
