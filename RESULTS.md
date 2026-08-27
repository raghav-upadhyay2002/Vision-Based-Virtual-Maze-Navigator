# Vision-Based Virtual Maze Navigator — Pipeline, Experiments and Findings

**Author:** Raghav Upadhyay
**Repository:** https://github.com/raghav-upadhyay2002/Vision-Based-Virtual-Maze-Navigator
**Demo video:** https://drive.google.com/file/d/1d2R76Qy9Kmbr4RCgXTRKCL5U3j1-cjew/view?usp=sharing
**Simulator:** Webots (e-puck robot)
**Date:** August 2026

---

## 1. Overview

This project implements an autonomous navigation controller for an e-puck robot in a Webots
maze world, using **camera input only**. The robot has no access to ground-truth pose, no
odometry-based mapping, and no distance sensors in the control loop — every navigation
decision is derived from pixels.

The goal was twofold:

1. Build a reproducible camera-only navigation demo that reliably reaches a red target in a
   maze.
2. Characterise how that perception pipeline degrades under changes in visual conditions
   (low illumination, blur, partial occlusion, unfamiliar environments), and fix the
   failures that appear.

The second goal turned out to be where the interesting work was. Most of the design
decisions in the final controller are direct responses to a specific observed failure, and
Section 4 documents each of those in the order they were encountered.

---

## 2. System Pipeline

### 2.1 Sensing

| Device | Webots name | Role in the control loop |
|---|---|---|
| Front camera | `camera` | Wall detection ahead / left / front-right; red target detection |
| Right camera | `camera_right` | Near-field wall detection on the right side (turn arbitration) |
| Left/right wheel motors | `left wheel motor`, `right wheel motor` | Velocity-controlled actuation (`setPosition(inf)`) |

All devices are enabled at the simulator's basic timestep, and the control loop runs once
per `robot.step(timestep)`.

Webots returns each frame as a flat BGRA byte buffer. Every iteration this is reshaped to
`(height, width, 4)` and converted to 3-channel BGR before any processing.

### 2.2 Perception — front camera (wall geometry)

```
BGRA frame → BGR → grayscale → GaussianBlur(5×5) → Canny(50, 150) → edge map
           → split into left / center / right zones (0–33%, 33–66%, 66–100%)
           → edge density per zone = nonzero_pixels / zone_area
           → threshold + temporal debounce → wall flags
```

The key design decision — explained in detail in **Problem 1** — is that a wall is
signalled by edge density **collapsing toward zero**, not by high edge density. A wall
close to the camera fills the frame as a flat, low-texture surface and produces almost no
gradients; open space with visible floor, sky and background scenery produces many.

The empirical cutoff is `density_epsilon = 0.005`, and each flag requires the condition to
hold for **3 consecutive frames** before it is reported (see **Problem 3**).

| Flag | Condition |
|---|---|
| `wall_ahead` | all three zones below epsilon for 3 frames |
| `wall_left` | left zone below epsilon **and** right zone above, for 3 frames |
| `wall_front_right` | right zone below epsilon **and** left zone above, for 3 frames |

`cv2.HoughLinesP` is computed on the edge map but is **not consumed** by the current control
logic. It is retained as scaffolding for a future line-based wall-geometry estimator.

### 2.3 Perception — right camera (near-field side wall)

```
Right camera frame → crop bottom 40% → grayscale → mean brightness → wall_right flag
```

Only the bottom 40% of the right camera frame is used, so the measurement reflects the wall
immediately beside the robot rather than walls further down the corridor (see
**Problem 5**).

The wall/no-wall decision is made from the mean brightness of that strip: a wall close to
the camera occludes the brighter floor and background, lowering the mean.

> **Note — threshold formulation.** The version currently on `main` uses a fixed absolute
> cutoff (`mean_right < 195.0`). This is the formulation that fails under low illumination
> (**Problem 6**); it has been replaced locally with a relative-change criterion
> (a brightness drop of more than ~30). *This section should be updated with the exact
> reference/baseline used once the revised code is pushed.*

### 2.4 Perception — target detection

```
BGR frame → HSV → two red hue ranges (0–7 and 170–180, S ≥ 120, V ≥ 70)
          → binary mask → pixel count
          → if count > 50: centroid x → 'left' (<40%) / 'center' / 'right' (>60%)
```

Red wraps around the hue wheel, so two `inRange` masks are combined to cover both ends of
the red band. HSV is used rather than BGR specifically because it separates chroma from
intensity, which was expected to give some illumination tolerance.

### 2.5 Control policy

A single-priority reactive policy — obstacle avoidance always dominates target seeking:

| Priority | Condition | Action |
|---|---|---|
| 1 | `wall_ahead` **and** `wall_right` | Turn left (L=0.0, R=3.0) |
| 2 | `wall_ahead` **and** right side clear | Turn right (L=3.0, R=0.0) |
| 3 | `wall_left` | Turn right (steer away) |
| 4 | `wall_front_right` | Turn left (steer away) |
| 5 | target visible, offset left | Turn left |
| 6 | target visible, offset right | Turn right |
| 7 | otherwise (target centered, or no target) | Drive straight (L=R=3.0) |

Priority 1 vs 2 — the right-camera arbitration of turn direction — is what breaks the
robot out of repeating loops (**Problem 4**).

### 2.6 Logging

Every control iteration appends a row to a timestamped CSV (`run_log_YYYYMMDD_HHMMSS.csv`)
next to the controller, flushed each frame so a run remains analysable even if the
simulation is interrupted:

`sim_time_s, wall_ahead, wall_left, wall_front_right, wall_right, left_density,
center_density, right_density, mean_right, target_visible, target_direction,
left_velocity, right_velocity`

This log is the primary evidence base for the robustness experiments — the illumination
finding in Section 5 comes directly from the `mean_right` column.

Four live OpenCV debug windows are also rendered each frame: the Canny edge map, the red
target mask (when visible), the front camera feed, and the cropped right-camera strip.

---

## 3. Experimental Setup

### 3.1 Environment

- Webots R2025a with the bundled e-puck robot, world file `e-puck/worlds/maze_path.wbt`
  (a 3 × 2 m `RectangleArena` containing 14 `Wall` nodes and a red `Ball` as the target)
- Controller: `e-puck/controllers/maze_nav/maze_nav.py`
- Cameras: front (the e-puck's built-in `camera`) plus `camera_left` / `camera_right`
  mounted in the turret slot, all 256 × 192 with a 0.84 rad field of view
- Lighting: `TexturedBackgroundLight` with `castShadows FALSE`; baseline `luminosity` is
  the default `1`
- Python dependencies: `opencv-python`, `numpy`, installed **in the interpreter Webots uses
  to launch controllers** (the `controller` module is not pip-installable and only exists
  inside a Webots-launched process)
- Maze world includes background scenery (sky, mountains) beyond the maze walls — this is
  not incidental, it is the source of **Problem 1**

### 3.2 Baseline

Baseline condition is the default world at **illumination = 1.0**. After the six fixes
documented in Section 4, the robot **navigates the maze and reaches the red target
successfully** under baseline conditions. A recording of a successful baseline run is
available here:
[demo video](https://drive.google.com/file/d/1d2R76Qy9Kmbr4RCgXTRKCL5U3j1-cjew/view?usp=sharing).

### 3.3 Visual conditions

| Condition | How it was induced | Status |
|---|---|---|
| Low illumination | `TexturedBackgroundLight.luminosity` reduced from 1.0 → 0.5 | Tested — failure found; fix specified (not yet on `main`) |
| Blur | `BLUR_ENABLED = True` in `maze_nav.py`: a 9 × 9 Gaussian applied to every camera frame before any processing | Tested — no degradation |
| Partial occlusion | Wall partially blocking the front camera's field of view | Tested — handled by existing zone logic |

### 3.4 Metrics

- **Per-frame:** all detector flags, the three front-camera zone densities, right-camera
  mean brightness, target visibility and direction, commanded wheel velocities (CSV log)
- **Per-run:** target reached (yes/no), qualitative failure mode (oscillation in place,
  looping without progress, collision, phantom turning)

> *Open item: the number of repeated trials per condition, and whether target-reach is
> detected programmatically (e.g. a red-pixel-count threshold terminating the run) or
> observed manually, should be recorded here.*

---

## 4. Problems Encountered and How They Were Addressed

These are in the order they were found. Each one changed the design.

### Problem 1 — Background scenery made "high edge density = wall" unusable

**Symptom.** The maze world contains scenery beyond the walls (sky, mountains). Canny
produced a large number of edges from that background, and the detector could not
distinguish "lots of edges because there's a textured wall in front of me" from "lots of
edges because I'm looking at a mountain range down an open corridor."

**Diagnosis.** Edge density is not monotonic in obstacle proximity when the background is
visually busy. A *distant* scene can easily out-produce a *near* wall in edge count.

**Fix.** Invert the signal. Instead of treating high edge density as the wall indicator,
treat density **collapsing to (near) zero** as the indicator. A wall close enough to matter
fills the frame as a flat, uniformly-lit, low-texture surface with almost no internal
gradients, whereas any normally-lit scene — however cluttered — retains contrast between
floor, sky and scenery and never zeroes out. This makes the detector immune to background
clutter by construction.

**Known limitation.** The fix is valid only while there is enough light to produce *some*
gradient. At (near) zero illumination the whole frame goes uniformly dark, edge density
collapses everywhere, and the detector reports "wall ahead" regardless of what is actually
in front of the robot. This is the front-camera analogue of Problem 5 — both are
illumination-dependent failures of an intensity-derived signal.

---

### Problem 2 — Partially-filling walls were missed

**Symptom.** When a wall occupied only part of the field of view — for example hugging the
left side while open floor remained visible elsewhere — the robot failed to register a wall
and drove into it.

**Diagnosis.** A single density measurement across the whole frame averages the flat wall
region together with the textured open region. The open region's edges dilute the average
enough that it never falls below the epsilon cutoff, so the flag never fires even though
the robot is about to clip the wall.

**Fix.** Split the front camera frame into **three vertical zones** (left, center, right)
and evaluate density independently in each. A wall filling only the left third now drives
the *left zone's* density to zero on its own, producing a `wall_left` flag and a turn away
from it, while the all-three-zones-zero case is reserved for a wall directly ahead.

---

### Problem 3 — Phantom turning from single-frame noise

**Symptom.** After the zone split, the robot began making spurious turns — changing
direction in response to walls that were not there.

**Diagnosis.** Per-zone density is noisier than a whole-frame average (smaller sample, so
frame-to-frame fluctuation matters more). Individual frames dipped below the epsilon cutoff
transiently, and because the controller acted on the flag immediately, every transient dip
became a steering command.

**Fix.** Temporal debouncing. Each zone maintains a counter of consecutive frames in which
its condition holds; a wall flag is only raised once the counter reaches **3**. Any frame
that fails the condition resets the counter to zero. This removes isolated false positives
at the cost of a ~3-frame detection latency, which is negligible at the robot's speed.

---

### Problem 4 — Looping without progress on a fixed turn rule

**Symptom.** With a fixed escape behaviour on `wall_ahead`, the robot would repeatedly
traverse the same path — turning the same way at the same junctions and returning to where
it started, making no net progress toward the target.

**Diagnosis.** A fixed turn direction is a deterministic policy in a deterministic
environment; in a maze it produces closed cycles. The robot had no information about
whether the direction it was about to turn into was actually open.

**Fix.** Add a **right-facing camera** and make the turn direction conditional on it. When
`wall_ahead` fires, the controller queries the right camera: if there is a wall on the
right, it turns **left**; if the right is clear, it turns **right**. The turn decision is
now driven by observed free space rather than a fixed rule, which breaks the cycles.

---

### Problem 5 — Right camera reacting to far walls

**Symptom.** The right-camera wall check reported a wall on the right when the nearest wall
on that side was actually well down the corridor, causing premature or incorrect turn
arbitration.

**Diagnosis.** The check was a mean-brightness measurement over the **whole** right-camera
frame. Under perspective projection, distant walls appear in the upper portion of the
frame; their darkness was being averaged into the same statistic used to decide whether a
wall was immediately adjacent. Near and far obstacles were indistinguishable in the metric.

**Fix.** Crop to the **bottom 40%** of the right camera frame before computing the mean
(`split_ratio = 0.4`). The bottom strip images the region immediately beside the robot,
which is the only part relevant to the turn decision, so the statistic now reflects
near-field occupancy only.

---

### Problem 6 — Low illumination broke the absolute brightness threshold

This is the first perturbation-condition failure, found by reducing world illumination from
1.0 to 0.5.

**Symptom.** Under reduced illumination the robot became stuck **oscillating in place**: it
would approach a wall, correctly detect `wall_ahead`, attempt to turn, and immediately
re-trigger, never escaping the location.

**Measurements (from the `mean_right` log column):**

| Illumination | `mean_right`, wall present | `mean_right`, no wall | Separation | Verdict under fixed threshold (195) |
|---|---|---|---|---|
| 1.0 (baseline) | ~170 | ~200+ | ~30+ | Correct — threshold sits between the two |
| 0.5 | ~130 | ~164 | ~34 | **Both below 195** → always reports "wall" |

**Diagnosis.** The *discriminative information survived* the illumination change — the gap
between wall and no-wall stayed roughly the same size (~30). What broke was the **absolute
reference**: dimming shifted both values down together, so a fixed cutoff calibrated at
illumination 1.0 now sat above both of them and classified everything as "wall."

The failure then propagated through the control policy. Because `wall_right` was
permanently true, the `wall_ahead` branch always took priority 1 (turn left) and never
priority 2 (turn right) — the arbitration added in Problem 4 was effectively disabled, and
the robot lost the ability to choose its escape direction. Notably, the **front camera was
unaffected**: Canny responds to local gradients rather than absolute intensity, so as long
as there is enough light for any contrast to exist, the zone densities behave normally.
The failure was isolated to the one detector that used an absolute intensity cutoff.

**Fix.** Replace the fixed absolute threshold with a **relative-change criterion**: a wall
is flagged when the brightness *drop* exceeds ~30, rather than when the absolute mean falls
below 195. Because the wall/no-wall separation is preserved under dimming while the
absolute level is not, keying on the change rather than the level makes the detector
invariant to global illumination shifts.

> *Open item: this section should state explicitly what the drop is measured against
> (startup baseline / rolling average of recent frames / another region of the same frame)
> once the revised code is pushed.*

---

## 5. Robustness Results

| Condition | Outcome | Explanation |
|---|---|---|
| **Baseline** (illumination 1.0) | Reaches target | Works after the six fixes above ([demo video](https://drive.google.com/file/d/1d2R76Qy9Kmbr4RCgXTRKCL5U3j1-cjew/view?usp=sharing)) |
| **Low illumination** (0.5) | Failed; fix specified, not yet on `main` | Absolute brightness threshold invalidated; oscillation in place. Addressed by a relative-change threshold (Problem 6) |
| **Blur** | No degradation | No fix required — see below |
| **Partial occlusion** | Handled | No new fix required — see below |

### Why blur did not degrade performance

Neither detector depends on high-frequency detail:

- The **right camera** check is a *mean brightness* statistic over a region. Blur
  redistributes intensity locally but leaves the regional mean essentially unchanged, so the
  wall/no-wall signal is untouched.
- The **front camera** check depends on the *presence* of gradients, not their sharpness.
  The wall/floor and wall/sky boundaries are large-scale intensity steps that survive
  blurring, and the pipeline already applies a 5×5 Gaussian blur before Canny — so
  additional blur is a difference of degree, not of kind, and edge density in open space
  stays comfortably above epsilon.

The zero-density-means-wall formulation (Problem 1) is what makes this hold: the pipeline
does not need to *count* edges accurately, only to distinguish "some structure" from "none."

### Why partial occlusion was already handled

Partial occlusion of the front camera by a wall is structurally the same situation as
Problem 2, and the three-zone split already solves it. An occluding surface zeroes the
density of the zone(s) it covers while the unobstructed zones retain their normal density,
which is exactly the `wall_left` / `wall_front_right` signature — the robot steers away
from the occluded side. **No additional mechanism was needed; this condition passes as a
direct consequence of the Problem 2 fix.**

---

## 6. Discussion

The recurring theme across all six problems is **which property of the image the decision is
keyed on**, and how that property behaves when conditions change:

- Signals keyed on **absolute intensity** (right-camera mean vs. a fixed cutoff) are
  fragile: they break as soon as the global illumination level moves, even though the
  underlying scene information is perfectly intact. This was the low-illumination failure.
- Signals keyed on **relative structure** (edge presence/absence, brightness *change*) are
  substantially more robust: they survived both the illumination change and blur without
  modification.

The single most valuable change in the project was therefore not an algorithmic addition
but a reformulation — moving the right-camera detector from an absolute to a relative
criterion. That same principle explains, in hindsight, why the front camera never failed in
the first place.

The second theme is that **spatial and temporal aggregation must be matched to the decision
being made**: averaging over too large a region hid partial walls (Problem 2) and confused
near with far (Problem 5), while acting on too short a time window produced phantom turns
(Problem 3).

---

## 7. Limitations and Future Work

**Current limitations**

- The controller is purely reactive — there is no map, no memory of visited locations, and
  no global planning. Escaping loops relies on local free-space arbitration rather than
  exploration state.
- Zero (or near-zero) illumination defeats the front-camera detector entirely
  (Problem 1's stated limitation).
- Target detection uses fixed HSV bounds with `V ≥ 70`; sufficiently dark conditions will
  push target pixels below that bound and the target will become invisible even while wall
  detection still works.
- Fixed empirical constants (`density_epsilon = 0.005`, `split_ratio = 0.4`, 3-frame
  debounce, target pixel count > 50) are tuned to this world and have not been validated
  across layouts.

**Natural next steps**
- Apply the same absolute→relative reformulation to the target detector, so it degrades
  gracefully under dimming.
- Add an explicit **recovery behaviour**: detect when the robot is oscillating or looping
  (both are visible in the existing CSV log as repeating velocity patterns) and trigger a
  deliberate escape rather than relying on the reactive policy to resolve it.
- Introduce a lightweight confidence measure per detector, so the controller can tell
  "I see no wall" from "I cannot see" — the distinction the low-illumination failure
  hinged on.

---

## 8. Reproducing These Results

1. Install Webots and ensure `numpy` + `opencv-python` are installed in the Python
   interpreter Webots uses for controllers (not just a project virtualenv).
2. Open `e-puck/worlds/maze_path.wbt`.
3. Confirm the e-puck node's `controller` field is set to `maze_nav`.
4. Press **Run**. Three OpenCV debug windows appear alongside the simulation (a fourth, the
   target mask, appears whenever the target is in view); a CSV log is written next to the
   controller.
5. To reproduce the low-illumination experiment, set `luminosity 0.5` on the world's
   `TexturedBackgroundLight` node and re-run. Compare the `mean_right` column of the
   resulting log against the baseline run — the wall/no-wall separation persists while the
   absolute level shifts down.
6. To reproduce the blur experiment, set `BLUR_ENABLED = True` at the top of `maze_nav.py`
   and re-run; the zone densities in the log stay comfortably above `density_epsilon`.
