# Vision-Based Virtual Maze Navigator

An autonomous mobile robot agent that navigates an unknown 3D maze environment using only a first-person virtual camera feed, built on the [Webots](https://cyberbotics.com/) robotics simulator with an e-puck robot.

## The Mission
The agent must locate a specific target object within a simulated environment. It operates without access to ground-truth map coordinates, relying entirely on real-time visual input from its onboard camera.

## Core Loop
1. **Capture**: Retrieve the current camera frame from the simulation.
2. **Perceive**: Detect walls, openings, and landmarks using computer vision.
3. **Map**: Update a local visual occupancy grid map.
4. **Plan**: Determine the next optimal movement (e.g., turn left, move forward).
5. **Act**: Execute the command in the simulator and repeat.

## Current Status
The autonomous loop above is the target design; the controller isn't there yet. Right now [`maze_nav.py`](e-puck/controllers/maze_nav/maze_nav.py) implements:
- Live camera feed display (OpenCV window) from the e-puck's onboard camera.
- Manual keyboard teleoperation (**W/A/S/D** to drive/turn, released keys stop the robot).
- An in-progress red-target color detector (HSV thresholding) that isn't wired into motor control yet.

Planning, mapping, and closed-loop autonomous control are not implemented yet.

## Project Structure
```
e-puck/
  controllers/
    maze_nav/
      maze_nav.py       # Robot controller: camera feed, teleop, target detection
  worlds/
    e-puck.wbt           # Webots world/scene (set the robot's `controller` field to "maze_nav")
  plugins/                # Webots-bundled plugin boilerplate (not project code, gitignored)
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
3. Press **Run** in Webots.
4. Use **W / A / S / D** to drive the robot manually while watching the live camera feed window.

## Key Learning Outcomes
* **Visual SLAM**: Simultaneous Localization and Mapping using pixel data.
* **Path Planning**: Autonomous navigation through unexplored spaces.
* **Closed-Loop Control**: Real-time feedback loops between perception and action.

## License
MIT — see [LICENSE](LICENSE).
