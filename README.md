# egrobots-line-navigation — Lane Navigation Without Wheel Odometry (ROS 2 Humble)

The rover travels from Point A to Point B, avoids obstacles that block it,
returns to its path afterwards, and stops on arrival — using **only a 2D LiDAR
and an IMU**. No wheel encoder information is used for localization.

Two briefs are implemented here, on the custom 4-wheel rover from the previous
tasks, with the same nodes and different parameter profiles:

| Brief | World | Launch | Written up in |
|---|---|---|---|
| Autonomous straight-line navigation (Week 4) | open ground, then a road lined with parked cars | `line_navigation.launch.py` | sections 1–9 |
| Autonomous greenhouse lane navigation | a greenhouse crop lane | `greenhouse.launch.py` | **section 10** |
| Sweeping the whole greenhouse, then parking on the charging bay | three crop lanes and a headland | `greenhouse.launch.py` + `greenhouse_mission_node` | **section 11** |

The second brief turned out to need very little new code: measuring your
position from two rows of parked cars and measuring it from two rows of plants
are the same problem. What it did need was giving up the assumption that the
lane is straight and of fixed width.

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

For the greenhouse lane, see section 10 — `greenhouse.launch.py` is this launch
with the greenhouse world and the greenhouse parameter profiles.

| Argument | Default | Effect |
|---|---|---|
| `world` | `egrobots_world.world` | `road_world.world` for the parked-car road, `greenhouse_world.world` for the crop lane |
| `nav_params` | `line_params.yaml` | Navigator profile in `config/`, e.g. `greenhouse_params.yaml` |
| `row_params` | `row_road.yaml` | Row localizer profile in `config/`, e.g. `row_greenhouse.yaml` |
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
| `row_localizer_node` | Fits the rows either side — parked cars or crop rows; publishes sideways position and heading for the EKF, the anchored centreline, and the lane measured at the rover |
| `greenhouse_mission_node` | Sweeps every lane in turn and parks on the charging bay, as a client of the `FollowLine` action (evaluation and demo) |
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

Re-run after the greenhouse changes (section 10) to check for regressions:
reached B, finished 0.2 cm from the lane, sideways estimator error 0.3 cm final
and 4.9 cm max, all obstacles avoided to the wide side.

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

---

## 10. Greenhouse lane navigation

A second brief: drive autonomously **inside a greenhouse lane**, keeping
centred between its two sides, correcting continuously, never getting too close
to either side, and coping with a lane that is not perfectly symmetrical.

```bash
ros2 launch egrobots_line_navigation greenhouse.launch.py
```

```bash
ros2 action send_goal -f /follow_line egrobots_line_interfaces/action/FollowLine "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 21.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
```

A and B only say where to start and where to stop. Where the path runs sideways
is decided by the crop rows.

### 10.1 Requirements

| ID | Requirement | Where |
|---|---|---|
| R1 | Operate inside a greenhouse lane | `greenhouse_world.world`, `greenhouse.launch.py` |
| R2 | Detect the boundaries of the lane | `row_localizer_node` — RANSAC + PCA line fit per side |
| R3 | Maintain a centred position | `lane_offset: 0.0` against the measured lane centre |
| R4 | Move forward autonomously | `FollowLine` action, no walk/pause cycle in this brief |
| R5 | Correct continuously when it drifts | cross-track control at 10 Hz against `/lane_centreline` |
| R6 | Never get too close to either side | `side_safety()` — push away and slow down |
| R7 | Cope with an asymmetric lane | width tracking, single-side fallback, local lane reference |

### 10.2 Research — how robots find their way down a crop lane

Four families of approach appear in greenhouse and field robotics:

1. **Physical guidance.** Commercial glasshouses run trolleys on the heating
   pipes, and many spraying robots follow a buried wire or a rail. Accurate and
   cheap, but the lane has to be built for the robot. Not applicable here.
2. **Absolute localization — GNSS or a prebuilt map.** RTK-GNSS is the standard
   in open fields and is useless under glass: the structure blocks and reflects
   the signal. The indoor equivalent is SLAM plus AMCL on a prebuilt map
   (`slam_toolbox`, Nav2). It works, but it localizes against a map of a crop
   that grows, is pruned, and is moved — the map is stale within weeks, and the
   lane you must stay centred in is not where the map says it is.
3. **Relative, reactive row following.** Detect the two rows in the current scan
   and steer by them. The common implementations are a Hough transform or a
   least-squares/RANSAC line fit on 2D LiDAR returns, sometimes on the vertical
   stems only; vision equivalents detect the vanishing point between rows. This
   is what agricultural row-following literature overwhelmingly uses, because
   the quantity you need — *where is the middle of this lane, right here* — is
   measured directly rather than inferred from a global pose.
4. **Dead reckoning between measurements.** Wheel odometry, IMU, or scan
   matching (`rf2o`, ICP) to carry position through gaps where the rows cannot
   be seen. Necessary as a complement, never sufficient alone: in a lane of
   near-identical plants, scan matching has little to lock onto along the lane.

**What was implemented: (3), backed by (4).** The rows are fitted per scan and
the rover steers by the lane centre measured at its own position; the EKF of
commanded velocity and IMU heading carries it across gaps. (2) was rejected
because a map of a growing crop goes stale, and because the task is defined
relative to the lane, not to a map. The pieces were already here from the road
brief — the same node fits parked cars and plant rows.

### 10.3 The simulated greenhouse

`worlds/greenhouse_world.world`, generated by `worlds/make_greenhouse_world.py`
with a fixed seed so it can be regenerated or retuned:

Four rows of plants make three lanes, each 1.6 m nominal, 2.05 m apart:

```
 row 3  +3.075   [plants] [plants] [plants] [plants]
 LANE B  +2.050   ←──────────────────────────────────
 row 2  +1.025   [plants] [plants]   ·· gap ··  [plants]
 LANE A   0.000   ──────────────────────────────────→   ← the rover starts here
 row 1  -1.025   [plants] [plants] [plants] [plants]
 LANE C  -2.050   ──────────────────────────────────→
 row 0  -3.075   [plants] [plants] [plants] [plants]

 x:        0        2 ────── the planted stretch ────── 22        26.3
       rover                                                  charging bay
```

* Each plant is its own rough box — jittered position, depth and height — so
  the LiDAR sees a ragged edge, not a wall. The LiDAR plane is 0.77 m above the
  floor and the plants stand 1.2–1.6 m, so it cuts through foliage.
* Every row's centre wanders ±0.08 m on its own period, so each lane's width
  **and** centre vary along its length: a lane's true centre is the mean of its
  two rows' centres, and its width runs between about **1.45 and 1.76 m**.
* Row 2 has a **2 m gap** (x = 11–13) where plants are missing, so a rover in
  lane A or lane B has to keep going on one side alone.
* Open headland at both ends for turning between lanes, and the **charging bay**
  in the far one: a pad painted on the floor with a board behind it. The pad has
  no collision — as a solid 2 cm box the rover wedged on its lip driving in.
* The rover starts **outside** the lanes and drives in.

### 10.4 What changed for a greenhouse

**The lane is measured locally, not anchored once.** On the road the path is a
straight line fixed at the start. A crop lane wanders, so `row_localizer_node`
also publishes `/lane_centreline` every scan — the centre between the two rows
*at the rover*, with their local direction — and the navigator steers by that
(`use_lane_centring`). This is what makes "centred between the two sides" mean
something when the two sides are not parallel.

**The stored width follows the measured one.** `width_tracking_gain` lets the
lane pinch and widen. The width check that protects the fit from a bad row
stays, but as a sanity check rather than a fixed expectation.

**The lane is anchored only when its sides are beside the rover.** A row seen
entirely from in front is seen end-on: the scan grazes the ends of the plants
rather than their faces, which reads the lane as wider than it is and its
direction as about a degree off. Anchoring from the lane's mouth produced a
lane direction 0.93° out, and since every later position is measured against
that line, the error grew with distance: **33 cm of estimator error by the end
of a 21 m lane.** Waiting until the rows are level with the rover
(`anchor_max_leading`) cut it to about 0.5°.

**Arrival is progress along the lane, not distance to a point.** With the
estimate off sideways by more than the goal tolerance, a rover judged by
distance to B can pass the end of the lane without ever coming within tolerance
of it — observed once, driving 5 m out of the lane before the stall detector
stopped it.

**Keeping off the sides (R6).** `side_safety()` measures the air beside the
rover's bodywork, left and right, from LiDAR returns alongside it. Inside
`min_side_clearance` the near side is added to the cross-track error — which the
controller already converts into a heading away from it — and the speed is
scaled down with the room left. There is no new state machine to get stuck in.
If **both** sides are inside `stop_side_clearance` the lane is narrower than the
rover should attempt and it stops instead of squeezing through.

### 10.5 Measured results

Scored against Gazebo ground truth. The lane's true centre is known from the
world generator, so deviation is measured against **the lane's actual centre at
each point**, not against y = 0. One run down a single lane:

| Metric | Result |
|---|---|
| Goal | **reached** |
| Deviation from the lane's true centre | **mean 3.3 cm, max 6.5 cm** |
| Clearance from the rover's side to the plants | never below **34 cm** (48 cm when perfectly centred) |
| Through the 2 m gap in row 2 | a few centimetres off centre; the fit carries on from one side |
| True lane width over the run | 1.45 – 1.76 m |

R6, tested separately by telling the rover to drive 35 cm off centre, which
would leave 13 cm of air beside the plants:

| | Without the limiter | With it |
|---|---|---|
| Held at | 35 cm off centre | **26 cm off centre** |
| Clearance to the plants | 13 cm | **21 cm mean** |

It is a proportional push, not a hard constraint: it trades off against the
cross-track term rather than stopping at a threshold, and in that run it still
touched **6 cm** at one moment. It reached the end of the lane either way.

### 10.6 What this does not do

- **The absolute estimate drifts sideways along the lane.** The lane's measured
  direction is about 0.5° out after anchoring, and `/row_pose` places the rover
  against that straight line, so the EKF's sideways estimate is out by ~22 cm by
  the end of 21 m. None of R1–R7 depend on it — they are all lane-relative, and
  the rover is physically centred to 3.5 cm — but any consumer of the absolute
  pose should know. The obvious fix, letting the stored lane direction follow
  the measured one, was **rejected**: the row heading is what bounds IMU drift,
  and slaving the lane to the IMU would make heading pure dead reckoning.
- **The lane must be straight.** Both the fit and the anchored reference assume
  it. A curved lane needs the straight-line fit replaced by a spline or an arc.
- **One lane per run.** Entering the lane, driving it and stopping at the end;
  turning at the end into the next lane is a separate problem.
- **Plants are rigid boxes.** Real foliage moves, is partly transparent to a
  LiDAR, and returns noisier ranges.
- **Results are one run per case,** enough to show the behaviour, not to put an
  error bar on it.

---

## 11. Sweeping the greenhouse

Driving one lane is the building block. The job is the whole greenhouse: every
lane in turn, then park on the charging bay.

```bash
ros2 launch egrobots_line_navigation greenhouse.launch.py
```

```bash
ros2 run egrobots_line_navigation greenhouse_mission_node
```

`greenhouse_mission_node` is a **client of the same `FollowLine` action**, not
new navigation code. The sweep is a list of legs:

| Leg | From → to | Steering |
|---|---|---|
| lane 1 | (0.5, 0.00) → (24.0, 0.00) | the measured lane |
| turn | (24.0, 0.00) → (24.0, 2.05) | the straight line between waypoints |
| lane 2 | (24.0, 2.05) → (0.5, 2.05) | the measured lane |
| turn | (0.5, 2.05) → (0.5, −2.05) | the straight line |
| lane 3 | (0.5, −2.05) → (24.0, −2.05) | the measured lane |
| line up, face, dock | across the headland, then onto the pad | the straight line |

Each lane leg runs headland to headland, so the rover enters and leaves every
lane in a straight line and the turns are pure sideways moves in the open. Two
things change per leg: on a lane leg the rover steers by the lane either side of
it, and the localizer is told to **forget the previous lane** first — its anchor
is one lane's geometry, and in the next lane it would look for rows where there
are none.

### 11.1 Two things that had to change, and why

**Legs are planned from where the rover believes it finished the last one, not
from absolute coordinates.** The estimate drifts sideways as it drives a lane —
about a centimetre per metre, from the error in the lane direction measured on
entry. Driving the next leg to a fixed coordinate therefore enters the next lane
off centre by that drift. Measured: it entered lane 2 **0.6 m off centre**, close
enough to the plants that the lane could not be measured at all, and ploughed
into the row. Planning each leg relative to the believed end of the last one
cancels the drift, because it is common to both ends of a sideways move.

**Docking is done by looking at the bay.** After a 70 m sweep the rover's
estimate of where it is has drifted by **two to three metres** — far more than
the pad is wide. The last legs line the rover up, turn it to face the bay, and
then it finds the **board behind the pad** in the LiDAR: the nearest run of
returns in a cone ahead, in an otherwise empty headland. It parks a board's
length short of it. Aimed by dead reckoning it stopped 1.1 m away; aimed by
sight, 0.36–0.44 m.

### 11.2 Measured results

One full sweep, scored against ground truth:

| Lane | Deviation from that lane's true centre | Closest the rover's side came to the plants |
|---|---|---|
| 1 (middle) | mean **3.3 cm**, max 6.5 cm | 34 cm |
| 2 (left) | mean **3.5 cm**, max 7.9 cm | 39 cm |
| 3 (right) | mean **2.2 cm**, max 5.2 cm | 38 cm |

| | |
|---|---|
| Duration | 336 s of sim time, about 70 m driven |
| Parked | **44 cm from the pad centre**, on a 1.6 × 1.2 m pad |
| Estimator error at the end | 2.9 m sideways, 0.2 m along |

The sweep works *because* nothing in it depends on that 2.9 m: steering is
relative to the lane, leg planning is relative to the last leg, and docking is
relative to a landmark in view.

### 11.3 Wheel encoders: what they were expected to do, and what they did

The straight-line brief forbade wheel feedback. The greenhouse brief does not,
so the drive controller's odometry is fused (`config/ekf_greenhouse.yaml`) —
forward speed from the wheels, heading still from the IMU, since encoder-derived
rotation on a skid-steer measured ~40% of commanded.

**They were added to fix the estimate running away when the rover is held up.
They do not.** Driven into a plant row and held there, with the clearance stop
disabled so the rover kept pushing:

| | Estimate's phantom travel while the rover sat still |
|---|---|
| Commanded velocity only | 11.64 m |
| With wheel encoders fused | **12.15 m** |

In Gazebo the wheels are driven through a velocity interface: a blocked rover's
wheels keep turning and the encoders report full speed. An encoder cannot tell
slip from motion. On real hardware a blocked motor stalls and they would help —
in this simulation they do not, and the claim should not be made from it. The
fusion is kept because it is the right input on the real robot and costs
nothing, not because this test proved it.

`scan_is_frozen()` in the navigator is the honest attempt at the problem: if the
rover is being driven but the view is not changing, it is held up. It catches a
rover pinned dead still. It does **not** catch the case above, where the rover is
pinned but rocking — yaw swinging over 25° — because the view changes plenty.
Catching that means comparing the motion the scans imply against the motion
commanded, which is scan matching.

### 11.4 What this does not do

- **A jammed rover can report success.** With the estimate running away, the
  along-the-line arrival test fires and the goal returns SUCCEEDED while the
  rover has not moved: measured, 12.2 m of phantom travel and a successful
  result from a rover sitting against a plant. This is the most serious thing
  outstanding, and the fix is the scan matching above, not another sensor.
- **The absolute estimate is poor in the greenhouse** — metres over a sweep,
  against centimetres in the road world. Ragged foliage gives a much noisier
  lane direction than flat car panels, and the error grows with distance driven.
  Nothing in the sweep depends on it; anything that did would need a landmark.
- **Steering by the lane always drives forwards along it.** The measured lane
  direction is taken to point the way the rover faces, so a goal behind the
  rover cannot be reached while lane centring is on. The mission avoids this by
  resetting the lane before each leg, so the rover pivots on the A→B line first.
- **One greenhouse, one layout, one run per case.** Enough to show the behaviour,
  not to put an error bar on it.
