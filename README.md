# Vision-Based Virtual Maze Navigator

An autonomous mobile robot that navigates a 3D maze and locates a target object using
**camera input only** — no ground-truth pose, no odometry map, no distance sensors in the
control loop. Built on the [Webots](https://cyberbotics.com/) simulator with an e-puck robot.

📄 **[Full write-up: pipeline, experiments, failures and fixes →](RESULTS.md)**

---

## What it does

The robot explores an unknown maze, avoids walls, and steers toward a red target — deciding
everything from pixels. Perception runs on three streams:

- **Front camera** → Canny edge density in three vertical zones → walls ahead / left / right
- **Right camera** → mean brightness of the near-field strip → wall immediately on the right
- **Front camera (HSV)** → red target detection and its horizontal offset

A reactive control policy resolves these into wheel velocities, prioritising obstacle
avoidance over target seeking.

## Status

**Working end to end.** The robot navigates the maze and reaches the target under baseline
conditions.

Robustness testing against changing visual conditions is in progress:

| Condition | Result |
|---|---|
| Baseline (illumination 1.0) | Reaches target |
| Low illumination (0.5) | Failure found → fixed (see [Problem 6](RESULTS.md#problem-6--low-illumination-broke-the-absolute-brightness-threshold)) |
| Blur | No degradation |
| Partial occlusion | Handled by existing zone logic |

## Key design decision

Counter-intuitively, a wall is detected by edge density **collapsing to zero**, not by high
edge density. The world contains background scenery (sky, mountains) that generates plenty
of edges, so "many edges" cannot mean "obstacle." A wall close enough to matter fills the
frame as a flat, low-texture surface and produces almost none.

This turned out to be why the front camera survived both the blur and low-illumination
tests unchanged — it keys on relative structure rather than absolute intensity. The one
detector that used an absolute threshold is the one that broke under dimming. That
contrast is the main finding of the project; [RESULTS.md](RESULTS.md) develops it in full.

## Project structure

```
e-puck/
  controllers/
    maze_nav/
      maze_nav.py        # Controller: perception, control policy, CSV logging
  worlds/
    e-puck.wbt           # Webots world (set the robot's `controller` field to "maze_nav")
  plugins/               # Webots-bundled boilerplate (not project code, gitignored)
requirements.txt
RESULTS.md               # Pipeline, experimental setup, results, problems and fixes
```

## Prerequisites

- [Webots](https://cyberbotics.com/) installed
- `numpy` and `opencv-python` installed **in the Python interpreter Webots runs controllers
  with** — not necessarily your project's `.venv`

```bash
pip install -r requirements.txt
```

> **Note:** Webots controllers import a `controller` module that only exists inside a
> Webots-launched process — it is not a pip package and is not visible from a normal
> terminal or virtualenv. Webots runs controller scripts with its own configured Python
> interpreter (by default, the system Python on `PATH`), so `numpy`/`opencv-python` must be
> installed there. You cannot run `python maze_nav.py` directly from a terminal.

## Running

1. Open `e-puck/worlds/e-puck.wbt` in Webots.
2. Select the e-puck node and confirm its `controller` field is set to `maze_nav`.
3. Press **Run**.

Four live OpenCV debug windows open alongside the simulation — the Canny edge map, the red
target mask, the front camera feed, and the cropped right-camera strip. A timestamped CSV
log (`run_log_YYYYMMDD_HHMMSS.csv`) is written next to the controller, recording every
detector flag, zone density, brightness reading and wheel command per frame.

## Reproducing the robustness experiments

To reproduce the low-illumination finding, reduce the world's light intensity from 1.0 to
0.5 and re-run. Compare the `mean_right` column against the baseline log: the wall/no-wall
separation persists (~30 either way) while the absolute level shifts down — which is
precisely why an absolute threshold fails and a relative one does not. Full protocol in
[RESULTS.md §3](RESULTS.md#3-experimental-setup).

## License

MIT — see [LICENSE](LICENSE).
