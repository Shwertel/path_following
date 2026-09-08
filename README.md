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
                                   ├──► EKF ──► odom → base_link
IMU heading (/imu/data) ───────────┘
```

The split follows what was measured on this rover against ground truth:

| Source | Forward distance | Rotation |
|---|---|---|
| Commanded velocity | **98–102% accurate** | ~40% (skid-steer scrub) |
| IMU | — | direct measurement, unaffected by scrub |

So forward velocity comes from the command we issued, and heading comes from the
IMU. Each supplies the quantity it is good at, and **no wheel is read**.

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

## 4. Build & Run

```bash
cd ~/ros2_ws
colcon build --packages-select egrobots_line_interfaces egrobots_line_navigation
source install/setup.bash
```

```bash
ros2 launch egrobots_line_navigation line_navigation.launch.py
```

```bash
ros2 action send_goal -f /follow_line egrobots_line_interfaces/action/FollowLine \
  "{start: {x: 0.0, y: 0.0, z: 0.0}, goal: {x: 30.0, y: 0.0, z: 0.0}, tolerance: 0.0}"
```

`-f` streams feedback: distance to goal, progress along the line, **signed
cross-track error**, phase (`WALKING` / `PAUSED` / `AVOIDING` / `RETURNING`), and
iteration count. `Ctrl+C` cancels the goal. Add `rviz:=false` to skip RViz.

Obstacles are not in the world; add them from the Gazebo GUI.

---

## 5. Nodes

| Node | Purpose |
|---|---|
| `line_navigator_node` | The `FollowLine` action server: line following, walk/pause cycle, avoidance, arrival |
| `cmd_prior_node` | Republishes the commanded velocity as a stamped, covariance-bearing twist for the EKF |
| `imu_relay_node` | Adds realistic covariances to Gazebo's IMU stream |
| `ekf_filter_node` | `robot_localization`, fusing the two into `odom → base_link` |

---

## 6. Measured Results

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

Absolute error grows, but **as a fraction of distance it stays near 0.5% and does
not accelerate**. All ten avoidance manoeuvres are invisible in the error column:
the rover swings 1.4 m off the line, returns, and the estimate never steps.

For contrast, the same rover in the previous task — with heading from wheel
odometry — lost **2.3 m to a single avoidance turn**. Removing wheel odometry
made return-to-line *more* accurate, not less, because IMU heading is unaffected
by the sideways scrub a skid-steer needs in order to turn.

---

## 7. Design Decisions

**The motion prior must be a continuous stream, not one message per command.**
`cmd_prior_node` originally published only when a command arrived. A Kalman
filter with no velocity measurement does not hold position — it propagates its
last estimate through the process model with nothing to correct it. The estimate
free-ran between command bursts and ran away entirely once commands stopped,
observed as the pose climbing past **50 m while the rover stood still**. It now
publishes at 20 Hz, repeating the last command if fresh and sending **explicit
zero** if it is older than `command_timeout`. "Nothing is driving the robot" is
not absence of information; it is the statement that velocity is zero.

**Gazebo's IMU publishes zero covariance.** A filter reads zero variance as
infinite confidence, which makes the update ill-conditioned. This went unnoticed
in earlier tasks because wheel odometry also supplied position and anchored the
filter; with wheel feedback removed there is no position observation at all and
the filter diverges outright. SDF's `<imu>` block has no orientation-noise field,
so the covariance cannot be set at the source — hence `imu_relay_node`.

**Everything runs on sim time.** `cmd_prior_node` stamping with the wall clock
while Gazebo stamped sim time gave the filter a `dt` of ~1.79 billion seconds
between inputs, integrating velocity across it to produce a 1.6e8 m estimate.
Every node in the launch now sets `use_sim_time`.

**Turn to face, then drive.** Beyond a heading error of 45° the rover pivots
rather than arcing. Rotation is where nearly all of this platform's estimation
error originates, so minimising *total* rotation matters more than the fact that
a pivot scrubs harder per degree.

**Obstacle avoidance outranks everything, including the pause.** It reads only
the LiDAR and never waits on the cycle timer or the estimator.

**Ground truth is published for measurement only.** The `p3d` plugin on
`/ground_truth/odom` exists so the estimator can be scored against reality. It is
not an input to the EKF, the controller or the navigator, and does not exist on
real hardware. It replaced polling the `gz model` CLI, which opened and closed a
Gazebo transport connection per sample and crashed `gzserver` outright
(`transport::Connection` assertion, SIGABRT) when sampled at 2.5 Hz.

---

## 8. Known Limitations

- **Dead reckoning has no absolute reference.** Error is a function of distance
  travelled and nothing ever removes any. 0.36% is small but unbounded: roughly
  2 m at 500 m, 4 m at a kilometre.
- **Scan matching would not help in this world.** `slam_toolbox` or `rf2o` bound
  error by aligning scans against environmental features — but in the empty world
  all 360 LiDAR rays return `inf`, and even with obstacles present the rover
  drives ~15 m blind between them. Sparse isolated boxes are poor matching
  geometry. Localization against a map is worth adding only in an environment
  with persistent structure (walls, a corridor); in open ground the honest answer
  is a different reference entirely — GNSS/RTK, fiducials, or a magnetometer.
- **The simulated IMU is better than a real one.** Gazebo derives it from ground
  truth plus configured noise. A physical IMU has gyro bias drift — its zero
  point wanders with temperature and time — which would add error growing with
  *time* rather than distance, and would show up worst on long runs.
- **The `cmd_vel` prior is open loop.** It cannot detect wheel slip, a stall, or
  the rover being pushed. True laser odometry (`rf2o`) would close that gap, at
  the cost of needing features to match against.
- **Avoidance is reactive, not planned.** A concave obstacle would trap the rover
  until the stall detector aborts.
- **No obstacles in the bundled world**; they are added by hand in Gazebo.
