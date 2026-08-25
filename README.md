# Vision-Based Virtual Maze Navigator

An autonomous mobile robot agent that navigates an unknown 3D maze using only its onboard camera feeds, built on the [Webots](https://cyberbotics.com/) robotics simulator with an e-puck robot.

## The Mission
The agent must navigate a simulated maze and locate a red target object. It operates without access to ground-truth map coordinates, relying entirely on real-time visual input from three onboard cameras (front, left, right).

## How It Works
[`maze_nav.py`](e-puck/controllers/maze_nav/maze_nav.py) runs a fully autonomous perceive-then-act loop every simulation timestep:

1. **Capture** — Grab the current frame from the front, left, and right cameras.
2. **Perceive walls** — Run Canny edge detection on the front camera frame, split it into left/center/right thirds, and use edge density in each zone as a proxy for "wall very close" (a flat, low-texture surface produces few edges). A wall must be reported for 3 consecutive frames before it counts, to debounce single-frame noise. The right camera's bottom strip is checked separately via mean brightness — a nearby wall darkens that region.
3. **Perceive target** — Threshold the front camera frame in HSV for red, and use the centroid of the red pixel mask to decide whether the target is to the left, right, or center of frame.
4. **Decide & act** — Priority order: avoid a wall ahead first (turning toward whichever side is open), then steer away from a wall hugging the left or right side, then steer toward a visible red target, otherwise drive straight. This right-hand-follow-ish scheme lets the robot both avoid collisions and home in on the target.

Debug windows (via OpenCV) show the live front/right camera feeds, the Canny edge map used for wall detection, and the red-pixel mask when the target is visible.

## Current Status
Implemented:
- Front/left/right camera capture and processing each control step.
- Edge-density-based wall detection (ahead / left / right) with frame-debouncing.
- Brightness-based right-wall detection from a cropped camera strip.
- HSV-based red target detection with left/center/right direction estimation.
- Closed-loop autonomous control combining wall avoidance and target seeking.

Not yet implemented:
- Use of the left camera in navigation logic (currently enabled but unused).
- Persistent mapping/occupancy grid or path planning beyond reactive, one-step-ahead decisions.
- Loop closure / stopping condition once the target is actually reached.

## Project Structure
```
e-puck/
  controllers/
    maze_nav/
      maze_nav.py       # Robot controller: wall + target vision, autonomous motor control
  worlds/
    e-puck.wbt           # Webots world/scene (robot's `controller` field is set to "maze_nav")
  plugins/                # Webots-bundled plugin boilerplate (not project code)
requirements.txt
```

## Prerequisites
- [Webots](https://cyberbotics.com/) installed.
- Python 3 with `numpy` and `opencv-python` installed **in the Python Webots itself runs controllers with** (see note below) — not necessarily your project's `.venv`.

```
pip install -r requirements.txt
```

> **Note:** Webots controllers import a `controller` module that only exists inside a Webots-launched process — it isn't a pip package and won't be visible from a normal terminal or virtualenv. Webots runs controller scripts with its own configured Python interpreter (by default, the system Python found in `PATH`), so `numpy`/`opencv-python` must be installed there, not just in `.venv`. You cannot run `python maze_nav.py` directly from a terminal; the script only runs when launched by Webots.

## Running
1. Open `e-puck/worlds/e-puck.wbt` in Webots.
2. Select the e-puck robot node and confirm its `controller` field is set to `maze_nav`.
3. Press **Run** in Webots and watch the robot navigate autonomously, with debug windows showing its camera feeds and vision processing.

## Key Learning Outcomes
* **Reactive vision-based navigation**: making steering decisions directly from processed camera pixels rather than ground-truth pose or a map.
* **Classical CV techniques**: HSV color thresholding, Canny edge detection, and density/brightness heuristics as lightweight alternatives to learned perception.
* **Closed-Loop Control**: real-time feedback between perception and motor commands, with simple temporal debouncing to reject noisy single-frame readings.

## License
MIT — see [LICENSE](LICENSE).
