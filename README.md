# egrobots-line-navigation — Straight-Line A→B Without Wheel Odometry (ROS 2 Humble)

The rover travels a straight line from Point A to Point B in walk/pause
iterations, avoids obstacles that block it, returns to the same line afterwards,
and stops on arrival — using **only a 2D LiDAR and an IMU**. No wheel encoder
information is used for localization.

Built for the Egrobots ROS 2 Week 4 Task (Autonomous Straight-Line Navigation),
on the custom 4-wheel rover from the previous tasks.

---

## 1. Requirements

| ID | Requirement | Where |
|---|---|---|
| R1 | Start from a defined Point A | Action goal `start` |
| R2 | Travel toward Point B in a straight line | `follow_line_step()` — cross-track control |
| R3 | Stop for 5 s after each walking iteration | `PAUSED` phase, `pause_duration` |
| R4 | Continue the cycle until the goal is reached | walk/pause loop in `execute_callback` |
| R5 | Detect obstacles in its path | forward LiDAR cone, `read_cone()` |
| R6 | Avoid any obstacle that blocks it | `avoidance_step()` — TURNING / CLEARING |
| R7 | Return to the same straight line afterwards | same cross-track controller as R2 |
| R8 | Continue toward B after returning | `CLEAR` → resumes the walk cycle |
| R9 | Determine arrival and stop | `goal_tolerance`, action result |

---

## 2. The constraint, and how it is honoured

The task forbids wheel encoder information. That removes the entire odometry
source used in earlier tasks. What is left is an IMU, a 2D LiDAR, and knowledge
of what we ourselves commanded.

```
cmd_vel (our own control output) ──┐
IMU heading (/imu/data) ───────────┼──► EKF ──► odom → base_link
car rows, measured by the LiDAR ───┘
  (road world only: sideways position + heading)
```

The split follows what was measured on this rover against ground truth:

| Source | Forward distance | Sideways position | Rotation |
|---|---|---|---|
| Commanded velocity | **98–102% accurate** | not observed | ~40% (skid-steer scrub) |
| IMU | — | — | direct measurement, unaffected by scrub |
| Car rows (LiDAR) | not observed | **direct, drift-free** | direct, drift-free |

Each source supplies the quantity it is good at, and **no wheel is read**.

`enable_odom_tf: false` stops the drive controller publishing `odom → base_link`,
and nothing in this package subscribes to `/diff_drive_controller/odom`.

**One honest caveat.** A four-wheel rover needs `ros2_control` to actuate at all
— the simple diff-drive Gazebo plugin refuses more than one joint per side — and
any simulated actuator closes a velocity loop on joint state internally. The
wheels are therefore read in order to *drive* them. What is guaranteed is that no
wheel-derived odometry reaches localization.

`cmd_vel` is our own control output, not a sensor. It is an open-loop prior: it
cannot notice the wheels slipping, stalling, or the rover being pushed.

---

## 3. The line is geometry, not waypoints

Given A and B:

```
u = (B - A) / |B - A|      direction along the line
n = (-u.y, u.x)            left-hand normal
r = P - A                  robot relative to A

along = r · u              progress toward B
cross = r · n              signed perpendicular distance from the line
```

The controller advances `along` while nulling `cross`. That single formulation
covers **both R2 and R7**: returning after an avoidance detour is not a special
manoeuvre, it is the same error being driven to zero from a larger starting
value. The steering correction is proportional to `cross`, capped at
`max_correction_deg`, so a rover on the line drives parallel to it and a
displaced one cuts back at an angle that grows with the displacement.

---

## 4. Measuring position against the parked cars

The deployment environment is a straight road lined with parked cars on both
sides. `road_world.world` models it: cars 4.5 × 1.8 × 1.5 m, 1 m apart, inner
edges at y = ±3.0 m (a 6 m lane), with one car missing from each row to leave
wider gaps.

The rover does not drive down the middle of it. Like a vehicle keeping to one
side of a road, it follows a **lane**: the measured centreline shifted sideways
by `lane_offset` (−1.5 m, the right-hand half of a 6 m lane). Only the path
moves — position is still measured against **both** rows, so the accuracy is the
same as driving down the centre.

```
 +3.9  [car][car][car] [car][car]   [car][car]      ← left row
  0.0  · · · · · · · · · · · · · · · · · · · ·      ← road centre (measured)
 -1.5  A ─────────── ▣ obstacle ──────────── B      ← path = lane (offset -1.5 m)
 -3.9  [car][car]   [car][car][car][car] [car]      ← right row
```

Every scan, `row_localizer_node`:

1. **Keeps only points near where each row should be.** It predicts each row's
   position from the current estimate and discards points more than
   `gate_width` (0.6 m) away. This excludes an obstacle the rover is swerving
   around, which otherwise lands on the same side as a row.
2. **Fits a straight line to each row** — RANSAC, then a least-squares refinement
   on the inliers. Gaps between cars just mean fewer points.
3. **Turns the two lines into two measurements.** With the road direction at
   angle `beta` in the robot frame and its left normal `nL`:

   ```
   s_L = nL · p_left   =  W/2 - e       offset of the left row
   s_R = nL · p_right  = -W/2 - e       offset of the right row

   e   = -(s_L + s_R) / 2               rover's offset left of the centreline
   psi = -beta                          rover's heading relative to the road
   ```

   If only one row passes the quality checks (≥12 inliers covering ≥2 m of
   road), that row and the known lane width are used instead, with doubled
   uncertainty. If neither does, nothing is published for that scan and the EKF
   coasts on the IMU and commanded velocity.

The road itself is **anchored in odom from the first scans**, while the rover is
still at its start pose and odom is exact: lane width, the rover's offset from
the centre, and the road's direction are measured, not hard-coded. The centreline
is published latched on `/road_centreline`, and the navigator projects A and B
onto it and shifts them sideways by `lane_offset` — **A and B say where to start
and stop; the car rows say where the path runs sideways**.

Driving in a lane puts one row close by, which changes how a detour must be
chosen: swerving towards the near row would drive into parked cars. Before
picking a side, the navigator checks the room abeam on each side against
`clearing_distance + side_clearance` and rules out a side without it. With room
on both sides — an open world, or a path down the middle of the road — nothing
is ruled out and the choice is the obstacle's position as before.

The measurement reaches the EKF as `/row_pose` with its covariance **rotated so
it is tight across the road and very loose along it** (σ 0.05 m across, 50 m
along). Parked cars pin down how far the rover is from the centreline, not how
far along the road it is.

Checked against ground truth before any full run, by forcing the rover through
the hard pivots that previously caused the slide:

| Manoeuvre | EKF sideways error vs truth |
|---|---|
| Pivot left 85°, drive to y ≈ 1.3 m, pivot back | 0.1–0.3 cm |
| 5 m straight while offset | 0.0 cm |
| Pivot right 85°, drive to y ≈ −0.3 m, pivot back | 0.0–0.2 cm |

The row measurement itself stayed within 1 mm and 0.03° of truth, including
while the rover sat side-on to the rows at ±92°.

---

## 5. Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select egrobots_line_interfaces egrobots_line_navigation
source install/setup.bash
```

In the road world (row correction on by default):

```bash
ros2 launch egrobots_line_navigation line_navigation.launch.py world:=road_world.world
```

Place obstacles at fixed positions, so repeated runs are comparable. Their `y`
is measured from the lane, not the road centre, so they stay in the rover's way:

```bash
ros2 run egrobots_line_navigation spawn_obstacles --ros-args -p layout:=road
```

| Layout | Obstacles (x, y from the lane) | Purpose |
|---|---|---|
| `road` | (8, −0.5), (18, −0.6), (27, −0.7) | Pushed towards the near row of cars: gaps of 0.5 / 0.4 / 0.3 m to the cars, too narrow for the rover, so the only way past is the wide side |
| `spread` | (8, 0.0), (18, +0.4), (27, −0.4) | On the lane and either side of it — the layout used for the earlier results |
| `centre` | (10, 0.0), (20, 0.0) | Two obstacles dead on the path |

If you change `lane_offset` in `config/line_params.yaml`, pass the same value
here (`-p lane_offset:=...`).

Send the goal:

```bash
ros2 action send_goal -f /follow_line egrobots_line_interfaces/action/FollowLine \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 30.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
```

`-f` streams feedback: distance to goal, progress along the line, **signed
cross-track error**, phase (`WALKING` / `PAUSED` / `AVOIDING` / `RETURNING`), and
iteration count. `Ctrl+C` cancels the goal.

Launch arguments:

| Argument | Default | Effect |
|---|---|---|
| `world` | `egrobots_world.world` | `road_world.world` for the parked-car road |
| `row_correction` | `true` | `false` keeps the road centreline but sends the EKF no row measurement — for comparison runs |
| `rviz` | `true` | `false` skips RViz |
| `gui` | `true` | `false` runs Gazebo headless (no OpenGL); physics and the CPU ray LiDAR are unaffected |

To record the estimate against ground truth while a goal runs:

```bash
ros2 run egrobots_line_navigation pose_logger_node --ros-args \
  -p use_sim_time:=true -p period:=0.05 -p csv_path:=/home/$USER/run.csv
```

---

## 6. Nodes

| Node | Purpose |
|---|---|
| `line_navigator_node` | The `FollowLine` action server: line following, walk/pause cycle, avoidance, arrival |
| `cmd_prior_node` | Republishes the commanded velocity as a stamped, covariance-bearing twist for the EKF |
| `imu_relay_node` | Adds realistic covariances to Gazebo's IMU stream |
| `row_localizer_node` | Fits the parked-car rows; publishes sideways position and heading for the EKF, and the road centreline |
| `ekf_filter_node` | `robot_localization`, fusing the inputs into `odom → base_link` |
| `pose_logger_node` | Prints/records the estimated pose beside ground truth (evaluation only) |
| `spawn_obstacles` | Spawns obstacles at fixed, named layouts (evaluation only) |

---

## 7. Measured Results

### Empty world — IMU and commanded velocity only

100 m line, five obstacles, ten avoidance manoeuvres, scored against Gazebo
ground truth:

| Metric | Result |
|---|---|
| Distance travelled | **104.79 m** |
| Final position error | **0.374 m — 0.36% of distance** |
| Max error over the run | 0.513 m |
| Peak deviation going around obstacles | 1.427 m |
| **True cross-track error at finish** | **0.288 m** |
| Obstacle encounters | 10 |

Error against distance:

```
travelled     err     err%
      5.9   0.059   1.00%
     24.4   0.133   0.54%
     48.9   0.224   0.46%
     75.9   0.441   0.58%
    104.6   0.374   0.36%
```

Absolute error grows, but as a fraction of distance it stays near 0.5%. Later
logged runs showed where the remaining sideways error comes from: not creep along
the straights, but **steps during the return turn after a detour** — one return
added ~20 cm, which then stayed flat for the rest of the run. That is the slide a
skid-steer makes while pivoting, which neither the IMU nor the commanded velocity
can observe.

For contrast, the same rover in the previous task — with heading from wheel
odometry — lost **2.3 m to a single avoidance turn**.

### Road world — with and without the car-row correction

30 m goal, three obstacles at identical fixed positions (the `spread` layout: on
the path, +0.4 m, −0.4 m), six runs alternated ON / OFF, scored against Gazebo ground truth. These
runs predate the lane offset, so the path was the road centreline. All six runs
reached B. The only difference between the two sets is whether `/row_pose`
reaches the EKF (`row_correction`); both follow the road centreline.

| Run | Worst sideways error added by one detour | Max estimator sideways error | Final estimator sideways error | **True deviation from path at finish** | True distance to B |
|---|---|---|---|---|---|
| ON 1  | 4.4 cm  | 8.2 cm  | 0.2 cm  | **2.2 cm**  | 17.0 cm |
| ON 2  | 5.5 cm  | 9.3 cm  | 0.1 cm  | **1.9 cm**  | 25.1 cm |
| ON 3  | 3.5 cm  | 6.8 cm  | 0.2 cm  | **3.2 cm**  | 9.0 cm  |
| OFF 1 | 9.6 cm  | 14.1 cm | 1.5 cm  | **2.2 cm**  | 28.0 cm |
| OFF 2 | 23.3 cm | 37.2 cm | 23.7 cm | **22.7 cm** | 43.1 cm |
| OFF 3 | 24.6 cm | 22.1 cm | 6.7 cm  | **7.9 cm**  | 85.4 cm |

| | ON (n=3) | OFF (n=3) |
|---|---|---|
| True deviation from path at finish, mean | **2.4 cm** (range 1.9–3.2) | **11.0 cm** (range 2.2–22.7) |
| Max estimator sideways error, mean | 8.1 cm | 24.5 cm |
| True distance to B, mean | 17.0 cm | 52.2 cm |

What the correction changes is not that the rover stops sliding — each detour
still briefly puts the estimate off by up to 5 cm — but that the error no longer
**persists**. With the rows fused, the final estimator error was 0.1–0.2 cm in
every run; without them it ranged up to 23.7 cm and was carried to the finish.
That shows up as consistency: every ON run finished within 3.2 cm of the path,
while OFF ranged from 2.2 cm (a run whose detour errors happened to cancel) to
22.7 cm.

Three runs per condition is a small sample, and the along-road distance to B is
not directly corrected by the rows, so its improvement should not be read as
more than an indication.

### Road world — driving in a lane

The same 30 m goal with the path offset 1.5 m to the right of the road centre
and the `spread` obstacles moved with it, row correction on, one run:

| Metric | Result |
|---|---|
| Goal | **SUCCEEDED** |
| True deviation from the lane at finish | **0.3 cm** |
| Sideways tracking of the lane, outside detours | mean 5.0 cm, max 29.8 cm |
| Sideways estimator error | final 0.5 cm, max 5.4 cm |
| Detour excursion off the lane | 1.85 m |
| Obstacle encounters | 7, **every one avoided to the left** — away from the near row |
| Closest approach to a car row | 1.44 m from the rover's centre (1.19 m from its side) |
| Along-road estimator error at finish | 59 cm — dead-reckoned, unchanged by the rows |

The rover starts on the road centre, merges into the lane within about 2 m of
travel, and holds it. Sideways accuracy is the same as it was down the middle:
both rows are still measured, one just sits closer. The side rule did its job —
with only 1.5 m of road to the right, all seven detours went left.

### Road world — obstacles at the kerb side

The `road` layout pushes the obstacles towards the near row of cars (0.5 / 0.4 /
0.3 m gaps, all too narrow for the rover). This is the hard case for the row
measurement as well as for avoidance: the obstacles sit inside the band where
the right-hand row is looked for, and hide part of it while the rover passes.
One run:

| Metric | Result |
|---|---|
| Goal | **SUCCEEDED** |
| True deviation from the lane at finish | **1.8 cm** |
| Sideways tracking of the lane, outside detours | mean 5.3 cm, max 29.9 cm |
| Sideways estimator error | final 0.5 cm, **max 4.8 cm** |
| Obstacle encounters | 6, **every one avoided to the wide side** |
| Closest the rover's body came to an obstacle | about 0.25 m |
| Closest the rover's side came to a parked car | 1.12 m |
| Row fit, sampled over the run | both rows found in every sample (28 of 28) |
| Along-road estimator error at finish | 35 cm — dead-reckoned, unchanged by the rows |

The rover never tried the gap between an obstacle and the cars, and the row
estimate stayed as accurate as on the open lane. That is expected rather than
proven here: an obstacle covers about a metre of road, the fit looks for a line
at least 2 m long, and RANSAC keeps the longer run of parked cars around it.

---

## 8. Design Decisions

**Measure the slide instead of trying to prevent it.** Slowing the return turn
(0.3 rad/s, 30° cut-back) did reduce the slide in simulation, but the real robot
is heavy and its motors cannot turn that slowly, so that change was reverted:
`return_angular_speed` and `return_max_correction_deg` now equal the normal
turning values. The car-row measurement removes the need — the rover turns at
full speed, slides, and the slide is measured and corrected within a scan or two.

**Sideways only, not a full pose.** Parked cars constrain the rover's distance
from the centreline and its heading, but a row of similar cars looks nearly the
same as you slide along it. Rather than trust a weak along-road estimate, the
measurement's covariance is rotated to be tight across the road and loose along
it, and the EKF keeps its own along-road estimate from commanded velocity.

**The road is measured at start-up, not hard-coded.** Lane width, direction and
the rover's starting offset are taken from the first scans, while odom is still
exact. The same code works for any straight road of any width.

**Row points are gated by prediction.** During a detour the obstacle can sit on
the same side as a row. Keeping only points near the predicted row line — then
fitting with RANSAC — stops it biasing the fit. If the two fitted rows disagree
about the lane width, the one closer to its prediction is kept.

**The path is a lane, not the middle of the road.** A and B are projected onto
the measured centreline and shifted sideways by `lane_offset`, so a goal given
anywhere near the road follows the lane. Offsetting the path rather than moving
the cars is what works on a real road, where the rows are where they are; it also
keeps the measurement honest, since both rows are still used to fix position.
Set `lane_offset: 0.0` to drive down the centre again.

**A detour never turns into the near row.** With the path 1.5 m off centre there
is 1.5 m of road on one side and 4.5 m on the other, and a detour moves the rover
roughly 1–1.3 m sideways. Choosing the side by nearest obstacle return alone
would swerve into the cars whenever the obstacle sat slightly the other way, so a
side with less than `clearing_distance + side_clearance` (2.0 m) of room abeam is
ruled out first. Room is read from the LiDAR, not from the stored road width, so
the rule also holds while the rover is already displaced.

**The motion prior must be a continuous stream, not one message per command.**
`cmd_prior_node` originally published only when a command arrived. A Kalman
filter with no velocity measurement does not hold position — it propagates its
last estimate through the process model with nothing to correct it. The estimate
free-ran between command bursts and ran away entirely once commands stopped,
observed as the pose climbing past **50 m while the rover stood still**. It now
publishes at 20 Hz, repeating the last command if fresh and sending **explicit
zero** if it is older than `command_timeout`.

**Gazebo's IMU publishes zero covariance.** A filter reads zero variance as
infinite confidence, which makes the update ill-conditioned. This went unnoticed
in earlier tasks because wheel odometry also supplied position and anchored the
filter; with wheel feedback removed the filter diverges outright. SDF's `<imu>`
block has no orientation-noise field, so the covariance cannot be set at the
source — hence `imu_relay_node`.

**Everything runs on sim time.** `cmd_prior_node` stamping with the wall clock
while Gazebo stamped sim time gave the filter a `dt` of ~1.79 billion seconds
between inputs, integrating velocity across it to produce a 1.6e8 m estimate.
Every node in the launch now sets `use_sim_time`.

**Rotate or translate — never both at once.** The rover pivots in place until
within `align_tolerance_deg` (4°) of the target bearing, then drives straight.
Driving while the heading is still changing integrates distance along an
out-of-date heading. A `cross_track_deadband` (5 cm) stops it pivoting for noise.

**Pivots have a minimum rate.** The pivot command is proportional to heading
error, so it shrinks as the rover lines up. Measured on this rover, a pure pivot
below about 0.1 rad/s does not turn it at all — the wheels cannot overcome the
scrub a skid-steer needs to rotate. With a 4° tolerance the command near the edge
is 1.5 × 0.075 ≈ 0.11 rad/s, right at that threshold: three test runs deadlocked
stationary with a 4.2–4.3° heading error until the stall detector aborted them.
`min_pivot_speed` (0.3 rad/s) keeps every pivot above breakaway. It also matches
the real robot, whose heavy chassis and motors cannot turn very slowly either.

**Obstacle avoidance outranks everything, including the pause.** It reads only
the LiDAR and never waits on the cycle timer or the estimator.

**Ground truth is published for measurement only.** The `p3d` plugin on
`/ground_truth/odom` exists so the estimator can be scored against reality. It is
not an input to the EKF, the controller or the navigator, and does not exist on
real hardware. It replaced polling the `gz model` CLI, which opened and closed a
Gazebo transport connection per sample and crashed `gzserver` outright
(`transport::Connection` assertion, SIGABRT) when sampled at 2.5 Hz.

---

## 9. Known Limitations

- **Along the road, position is still dead-reckoned.** The car rows bound
  sideways error and heading, but not how far along the road the rover is, so
  where it stops relative to B keeps the accuracy of the IMU and commanded
  velocity alone. Using individual cars or gaps as landmarks would be the next
  step.
- **The row measurement assumes a straight road with parallel, parked rows.**
  Curves, angled parking or moving vehicles would break the straight-line fit.
- **If both rows are hidden or missing, the correction pauses.** The EKF coasts
  on the IMU and commanded velocity until a row is seen again. In a world with no
  rows at all the road never anchors, the localizer sends nothing, and the
  navigator follows A→B exactly as before.
- **The simulated IMU is better than a real one.** Gazebo derives it from ground
  truth plus configured noise. A physical IMU has gyro bias drift; on the road the
  row heading measurement bounds that too, but in open ground it would not.
- **Simulated LiDAR scans are instantaneous.** A real spinning LiDAR skews a scan
  taken during a fast pivot, which would add error to the row fit at exactly the
  moments it matters most; de-skewing with IMU yaw rate would address it.
- **Avoidance is reactive, not planned.** A concave obstacle would trap the rover
  until the stall detector aborts.
- **The lane offset is a fixed number, not a lane the rover reads.** It assumes
  the lane it should drive in sits a set distance from the centre between the
  rows. Nothing looks for lane markings, and an obstacle parked across the whole
  wide side would leave the rover with no side to swerve to.
