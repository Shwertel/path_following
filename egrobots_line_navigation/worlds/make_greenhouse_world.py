"""Generate greenhouse_world.world.

A greenhouse lane is not a corridor with flat walls. Its sides are rows of
plants: a broken, bumpy surface whose edge wanders, with gaps where a plant is
missing or dead. This generator builds that, deterministically (fixed seed), so
the world can be regenerated or retuned:

    python3 make_greenhouse_world.py > greenhouse_world.world

Layout, all distances in metres:

  * The lane runs along +x from x = 2 (its mouth) to x = 22 (its end). The rover
    spawns at the origin, outside the lane, and drives in.
  * Its two sides are plant rows whose inner faces sit nominally 0.8 either side
    of y = 0 - a 1.6 m lane - and wander by +/-0.1 along the lane, so the true
    width varies between about 1.4 and 1.8 m and its centre wanders with it.
  * The left row has a 2 m gap (x = 11..13) where plants are missing, so the
    rover must keep going on one side alone.
  * Each plant is a rough box: jittered width, depth, height and inner face, so
    the LiDAR sees a ragged edge rather than a straight wall.
  * Two more rows further out make the neighbouring lanes, to check that the
    localizer locks onto the right pair.
  * The glasshouse shell is far enough out not to enter the fitting window.

Everything is static; the rows are one model each, so Gazebo has a handful of
bodies rather than hundreds.
"""

import math
import random

LANE_START, LANE_END = 2.0, 22.0        # the planted stretch
HALF_WIDTH = 0.8                        # nominal inner face of each row
ROW_DEPTH = 0.8                         # how far the plants extend outwards
SPACING = 0.5                           # plant pitch along the row
GAP = (11.0, 13.0)                      # missing plants in the left row
NEIGHBOUR = 2.4                         # centre spacing between lanes


def wander(x, amplitude, period, phase):
    """Slow variation of a row's inner face along the lane."""
    return amplitude * math.sin(2.0 * math.pi * x / period + phase)


def plants(rng, inner_sign, centre_offset, period, phase, gap=None):
    """One row of plants as a list of (x, y, sx, sy, sz) boxes.

    inner_sign is +1 for a row on the left of its lane, -1 on the right;
    centre_offset shifts the whole row to serve a neighbouring lane.
    """
    boxes = []
    x = LANE_START
    while x < LANE_END:
        if gap is None or not (gap[0] <= x <= gap[1]):
            face = HALF_WIDTH + wander(x, 0.10, period, phase) + rng.uniform(-0.03, 0.03)
            depth = rng.uniform(0.35, 0.50)
            length = rng.uniform(0.35, 0.48)
            height = rng.uniform(1.2, 1.6)
            y = inner_sign * (face + depth / 2.0) + centre_offset
            boxes.append((x + rng.uniform(-0.04, 0.04), y, length, depth, height))
        x += SPACING
    return boxes


def row_model(name, boxes, colour):
    parts = []
    for i, (x, y, sx, sy, sz) in enumerate(boxes):
        parts.append(f"""      <collision name="c{i}"><pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name="v{i}"><pose>{x:.3f} {y:.3f} {sz/2:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>""")
    body = "\n".join(parts)
    return f"""    <model name="{name}">
      <static>true</static>
      <link name="row">
{body}
      </link>
    </model>"""


def wall(name, x, y, sx, sy, sz=2.5):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {sz/2} 0 0 0</pose>
      <link name="body">
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.8 0.85 0.85 0.4</ambient><diffuse>0.8 0.85 0.85 0.4</diffuse></material></visual>
      </link>
    </model>"""


def main():
    rng = random.Random(7)
    green = "0.15 0.45 0.15 1"
    dark = "0.12 0.35 0.12 1"
    models = [
        # The two sides of the lane the rover drives down.
        row_model('crop_row_left', plants(rng, +1, 0.0, 9.0, 0.0, gap=GAP), green),
        row_model('crop_row_right', plants(rng, -1, 0.0, 7.0, 1.1), green),
        # The far sides of the neighbouring lanes.
        row_model('crop_row_left_outer', plants(rng, +1, 2 * NEIGHBOUR, 8.0, 2.0), dark),
        row_model('crop_row_right_outer', plants(rng, -1, -2 * NEIGHBOUR, 6.5, 0.4), dark),
        # Glasshouse shell, kept clear of the fitting window.
        wall('wall_near', -5.0, 0.0, 0.2, 16.0),
        wall('wall_far', 30.0, 0.0, 0.2, 16.0),
        wall('wall_left', 12.5, 8.0, 35.0, 0.2),
        wall('wall_right', 12.5, -8.0, 35.0, 0.2),
    ]
    print(f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <physics type="ode">
      <ode><solver><iters>150</iters></solver></ode>
    </physics>
    <!-- Greenhouse lane along +x. Generated by make_greenhouse_world.py:
         plant rows with inner faces nominally at y = +/-0.8 (a 1.6 m lane),
         wandering +/-0.1 along the lane, with a 2 m gap in the left row at
         x = 11..13 and neighbouring lanes 2.4 m either side. -->
{chr(10).join(models)}
  </world>
</sdf>""")


if __name__ == '__main__':
    main()
