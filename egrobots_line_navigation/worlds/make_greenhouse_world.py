"""Generate greenhouse_world.world.

A greenhouse lane is not a corridor with flat walls. Its sides are rows of
plants: a broken, bumpy surface whose edge wanders, with gaps where a plant is
missing or dead. This generator builds that, deterministically (fixed seed), so
the world can be regenerated or retuned:

    python3 make_greenhouse_world.py > greenhouse_world.world

Geometry, all distances in metres. Four rows make three lanes:

    row 3   +3.075 +/- wander     ]  depth 0.45
    LANE B  +2.050                ]  1.6 m nominal
    row 2   +1.025 +/- wander
    LANE A   0.000                   <- the rover starts facing down this one
    row 1   -1.025 +/- wander
    LANE C  -2.050
    row 0   -3.075 +/- wander

Each row's centre wanders along the lane on its own period, so every lane's
width AND centre vary: a lane's centre is the mean of its two rows' centres,
and its width is 1.6 m plus the difference between their wanders, which works
out between about 1.45 and 1.75 m.

  * Rows run from x = 2 (the lanes' mouths) to x = 22. Outside that, at both
    ends, is open headland for turning between lanes.
  * Each plant is a rough box - jittered position, depth and height - so the
    LiDAR sees a ragged edge rather than a wall.
  * Row 2 has a 2 m gap (x = 11..13) where plants are missing, so a rover in
    lane A or lane B must keep going on one side alone.
  * The charging bay sits in the far headland, where a sweep of three lanes
    finishes.
  * The glasshouse shell is far enough out not to enter the fitting window.

Everything is static, and each row is a single model, so Gazebo has a handful
of bodies rather than hundreds.
"""

import math
import random

LANE_START, LANE_END = 2.0, 22.0        # the planted stretch
LANE_WIDTH = 1.6                        # nominal gap between two rows
ROW_DEPTH = 0.45                        # nominal depth of a row of plants
SPACING = 0.5                           # plant pitch along the row
WANDER = 0.08                           # how far a row's centre wanders
GAP = (11.0, 13.0)                      # missing plants, row 2

PITCH = LANE_WIDTH + ROW_DEPTH          # 2.05 m from one row to the next
ROWS = [-1.5 * PITCH, -0.5 * PITCH, 0.5 * PITCH, 1.5 * PITCH]
LANES = [-PITCH, 0.0, PITCH]            # lane C, lane A, lane B
PERIODS = [6.5, 9.0, 7.0, 8.0]          # one per row, so no two wander alike
PHASES = [0.4, 0.0, 1.1, 2.0]

BAY = (26.3, 0.0)                       # charging bay, in the far headland
STANDOFF = 1.1                          # pad centre to the board behind it


def row_centre(row, x):
    """Where row `row` sits sideways at x - the thing a lane is measured from."""
    return ROWS[row] + WANDER * math.sin(2.0 * math.pi * x / PERIODS[row] + PHASES[row])


def lane_centre(lane, x):
    """A lane's true centre: midway between its two rows, wander and all."""
    return (row_centre(lane, x) + row_centre(lane + 1, x)) / 2.0


def plants(rng, row):
    """One row as a list of (x, y, sx, sy, sz) boxes."""
    boxes = []
    x = LANE_START
    while x < LANE_END:
        if not (row == 2 and GAP[0] <= x <= GAP[1]):
            depth = ROW_DEPTH + rng.uniform(-0.05, 0.05)
            length = rng.uniform(0.35, 0.48)
            height = rng.uniform(1.2, 1.6)
            y = row_centre(row, x) + rng.uniform(-0.03, 0.03)
            boxes.append((x + rng.uniform(-0.04, 0.04), y, length, depth, height))
        x += SPACING
    return boxes


def box(name, x, y, z, sx, sy, sz, colour, solid=True):
    collision = (f'<collision name="c"><geometry><box>'
                 f'<size>{sx} {sy} {sz}</size></box></geometry></collision>'
                 if solid else '')
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="body">
        {collision}
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>
      </link>
    </model>"""


def row_model(name, boxes, colour):
    parts = []
    for i, (x, y, sx, sy, sz) in enumerate(boxes):
        parts.append(f"""      <collision name="c{i}"><pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name="v{i}"><pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>""")
    return f"""    <model name="{name}">
      <static>true</static>
      <link name="row">
{chr(10).join(parts)}
      </link>
    </model>"""


def main():
    rng = random.Random(7)
    shades = ["0.12 0.35 0.12 1", "0.15 0.45 0.15 1",
              "0.15 0.45 0.15 1", "0.12 0.35 0.12 1"]
    models = [row_model(f'crop_row_{i}', plants(rng, i), shades[i]) for i in range(4)]
    models += [
        # Charging bay: a pad the rover parks on, and a stop board behind it.
        # Paint on the floor, not a kerb: as a solid box the rover wedged on
        # its 2 cm lip on the way in and sat there until the stall detector
        # gave up. A real charging bay is a marking and a connector.
        box('charging_pad', BAY[0], BAY[1], 0.01, 1.6, 1.2, 0.02,
            "0.9 0.6 0.1 1", solid=False),
        # The board is the landmark the rover docks on: it stands well above
        # the LiDAR plane (0.77 m) and is the only thing in the headland.
        box('charging_post', BAY[0] + STANDOFF, BAY[1], 0.6, 0.12, 1.2, 1.2, "0.3 0.3 0.35 1"),
        # Glasshouse shell, kept clear of the fitting window.
        box('wall_near', -5.0, 0.0, 1.25, 0.2, 16.0, 2.5, "0.8 0.85 0.85 0.4"),
        box('wall_far', 30.0, 0.0, 1.25, 0.2, 16.0, 2.5, "0.8 0.85 0.85 0.4"),
        box('wall_left', 12.5, 8.0, 1.25, 35.0, 0.2, 2.5, "0.8 0.85 0.85 0.4"),
        box('wall_right', 12.5, -8.0, 1.25, 35.0, 0.2, 2.5, "0.8 0.85 0.85 0.4"),
    ]
    print(f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode">
      <ode><solver><iters>150</iters></solver></ode>
    </physics>
    <!-- Greenhouse, generated by make_greenhouse_world.py. Four rows of plants
         make three {LANE_WIDTH} m lanes at y = {LANES[0]:+.2f}, {LANES[1]:+.2f} and {LANES[2]:+.2f}, each row's
         centre wandering so every lane changes width and centre along its
         length. Row 2 has a gap at x = {GAP[0]:.0f}..{GAP[1]:.0f}. Charging bay at
         ({BAY[0]}, {BAY[1]}). -->
{chr(10).join(models)}
  </world>
</sdf>""")


if __name__ == '__main__':
    main()
