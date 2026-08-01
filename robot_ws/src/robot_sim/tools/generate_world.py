#!/usr/bin/env python3
"""Generate the 30 m x 30 m indoor Gazebo world used by the robot_sim package.

Everything is built from primitive shapes (box / cylinder / sphere) so the
world file is completely self-contained: no model database download and no
external meshes are required.

Building layout (world frame, metres, origin at the centre of the building):

    +y (north)
     |
     +--------------------------------------------------+  y = +15
     |  SERVER  |  RECEPTION  |    OPEN-PLAN    | MEETING|
     |   ROOM   |  & LOUNGE   |     OFFICE      |  ROOM  |
     |----------+             |                 |        |
     |          |             |                 |        |
     +=====D====+======D======+========D========+===D====+  y = +1.75
     |             MAIN  CORRIDOR                        |
     +=====D====+======D======+========D========+===D====+  y = -1.75
     |                        |                 |        |
     |      WAREHOUSE         |    CAFETERIA    | ROBOTICS
     |      / STORAGE         |                 |   LAB  |
     +--------------------------------------------------+  y = -15
   x = -15                                            x = +15

  'D' marks a doorway.  Seven enclosed rooms in total, all opening onto the
  central corridor (the server room opens into the reception area).

Usage:  python3 generate_world.py [output.world]
"""

import math
import os
import sys

# ----------------------------------------------------------------------------
# Building dimensions
# ----------------------------------------------------------------------------
HALF = 15.0          # building is 30 m x 30 m
WALL_T = 0.2         # wall thickness
WALL_H = 2.6         # wall height
DOOR_W = 2.2         # doorway clear width (double door: a 0.5 m skid-steer
                     # base needs room to swing through without clipping)
CORR = 1.75          # corridor half width (corridor is 3.5 m wide)

MATERIAL_URI = "file://media/materials/scripts/gazebo.material"


# ----------------------------------------------------------------------------
# Low level SDF helpers
# ----------------------------------------------------------------------------
def _pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"


class LinkSet:
    """Collects the links of one static model, in the model's local frame."""

    def __init__(self):
        self.links = []

    def _add(self, geometry, pose, material, collide=True):
        name = f"link_{len(self.links)}"
        collision = ""
        if collide:
            collision = (
                f'      <collision name="{name}_collision">\n'
                f"        <geometry>{geometry}</geometry>\n"
                f"      </collision>\n"
            )
        self.links.append(
            f'    <link name="{name}">\n'
            f"      <pose>{pose}</pose>\n"
            f"{collision}"
            f'      <visual name="{name}_visual">\n'
            f"        <geometry>{geometry}</geometry>\n"
            f"        <material><script>"
            f"<uri>{MATERIAL_URI}</uri>"
            f"<name>Gazebo/{material}</name>"
            f"</script></material>\n"
            f"      </visual>\n"
            f"    </link>\n"
        )

    def box(self, sx, sy, sz, x=0, y=0, z=0, yaw=0.0, material="Grey", collide=True):
        geom = f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"
        self._add(geom, _pose(x, y, z, 0, 0, yaw), material, collide)

    def cylinder(self, radius, length, x=0, y=0, z=0, roll=0.0, pitch=0.0,
                 material="Grey", collide=True):
        geom = (f"<cylinder><radius>{radius:.4f}</radius>"
                f"<length>{length:.4f}</length></cylinder>")
        self._add(geom, _pose(x, y, z, roll, pitch, 0), material, collide)

    def sphere(self, radius, x=0, y=0, z=0, material="Grey", collide=True):
        geom = f"<sphere><radius>{radius:.4f}</radius></sphere>"
        self._add(geom, _pose(x, y, z), material, collide)


class World:
    """Accumulates static models."""

    def __init__(self):
        self.models = []

    def add(self, name, links, x=0.0, y=0.0, z=0.0, yaw=0.0):
        base = name
        i = 1
        while any(m.startswith(f'  <model name="{name}">') for m in self.models):
            i += 1
            name = f"{base}_{i}"
        body = "".join(links.links)
        self.models.append(
            f'  <model name="{name}">\n'
            f"    <static>true</static>\n"
            f"    <pose>{_pose(x, y, z, 0, 0, yaw)}</pose>\n"
            f"{body}"
            f"  </model>\n"
        )


# ----------------------------------------------------------------------------
# Structure: walls
# ----------------------------------------------------------------------------
def wall_segments(start, end, gaps):
    """Split the interval [start, end] into solid pieces around `gaps`.

    `gaps` is a list of (centre, width) doorways.  Returns (centre, length)
    pairs for the remaining solid wall pieces.
    """
    cuts = sorted((c - w / 2.0, c + w / 2.0) for c, w in gaps)
    pieces = []
    cursor = start
    for lo, hi in cuts:
        if lo > cursor:
            pieces.append(((cursor + lo) / 2.0, lo - cursor))
        cursor = max(cursor, hi)
    if end > cursor:
        pieces.append(((cursor + end) / 2.0, end - cursor))
    return pieces


def add_wall_x(world, name, y, x0, x1, gaps=(), material="PaintedWall"):
    """Wall running along the x axis at a fixed y."""
    ls = LinkSet()
    for cx, length in wall_segments(x0, x1, gaps):
        ls.box(length, WALL_T, WALL_H, cx, y, WALL_H / 2.0, material=material)
    world.add(name, ls)


def add_wall_y(world, name, x, y0, y1, gaps=(), material="PaintedWall"):
    """Wall running along the y axis at a fixed x."""
    ls = LinkSet()
    for cy, length in wall_segments(y0, y1, gaps):
        ls.box(WALL_T, length, WALL_H, x, cy, WALL_H / 2.0, material=material)
    world.add(name, ls)


# ----------------------------------------------------------------------------
# Furniture builders (all coordinates local to the model)
# ----------------------------------------------------------------------------
def desk(ls, w=1.6, d=0.8, h=0.75, top="Wood"):
    ls.box(w, d, 0.05, 0, 0, h - 0.025, material=top)
    for sx in (-1, 1):
        for sy in (-1, 1):
            ls.box(0.06, 0.06, h - 0.05,
                   sx * (w / 2 - 0.08), sy * (d / 2 - 0.08), (h - 0.05) / 2,
                   material="DarkGrey")


def round_table(ls, radius=0.7, h=0.75, top="White"):
    ls.cylinder(radius, 0.05, 0, 0, h - 0.025, material=top)
    ls.cylinder(0.08, h - 0.05, 0, 0, (h - 0.05) / 2, material="DarkGrey")
    ls.cylinder(0.35, 0.04, 0, 0, 0.02, material="DarkGrey")


def chair(ls, seat="Blue"):
    ls.box(0.45, 0.45, 0.06, 0, 0, 0.44, material=seat)          # seat
    ls.box(0.45, 0.06, 0.5, 0, -0.2, 0.72, material=seat)        # back rest
    ls.box(0.34, 0.34, 0.42, 0, 0, 0.21, material="DarkGrey")    # pedestal


def person(ls, shirt="Blue", trousers="DarkGrey"):
    ls.box(0.34, 0.24, 0.85, 0, 0, 0.425, material=trousers)     # legs
    ls.box(0.40, 0.26, 0.55, 0, 0, 1.15, material=shirt)         # torso
    ls.cylinder(0.06, 0.55, 0, 0.23, 1.15, material=shirt)       # left arm
    ls.cylinder(0.06, 0.55, 0, -0.23, 1.15, material=shirt)      # right arm
    ls.sphere(0.115, 0, 0, 1.55, material="Wood")                # head


def sofa(ls, w=2.0, colour="Purple"):
    ls.box(w, 0.85, 0.4, 0, 0, 0.2, material=colour)             # seat block
    ls.box(w, 0.2, 0.5, 0, -0.33, 0.65, material=colour)         # back
    for sx in (-1, 1):
        ls.box(0.2, 0.85, 0.3, sx * (w / 2 - 0.1), 0, 0.55, material=colour)


def plant(ls):
    ls.cylinder(0.22, 0.36, 0, 0, 0.18, material="Orange")
    ls.cylinder(0.05, 0.4, 0, 0, 0.5, material="Wood")
    ls.sphere(0.36, 0, 0, 0.95, material="Green")


def cabinet(ls, w=0.9, d=0.5, h=1.4, colour="DarkGrey"):
    ls.box(w, d, h, 0, 0, h / 2, material=colour)
    for k in range(3):
        ls.box(w * 0.8, 0.03, 0.04, 0, -d / 2 - 0.01, 0.3 + k * 0.4,
               material="Grey", collide=False)


def rack(ls, w=4.0, d=1.0, h=2.2, levels=3, colour="Grey"):
    """Warehouse pallet rack: 4 uprights + horizontal shelves."""
    for sx in (-1, 1):
        for sy in (-1, 1):
            ls.box(0.1, 0.1, h, sx * (w / 2 - 0.05), sy * (d / 2 - 0.05), h / 2,
                   material=colour)
    for k in range(levels):
        ls.box(w, d, 0.06, 0, 0, 0.1 + k * (h - 0.3) / max(levels - 1, 1),
               material="DarkGrey")


def crate(ls, s=0.7, colour="WoodPallet"):
    ls.box(s, s, s, 0, 0, s / 2, material=colour)


def counter(ls, w=6.0, d=0.8, h=1.05, colour="Wood"):
    ls.box(w, d, h, 0, 0, h / 2, material=colour)
    ls.box(w + 0.1, d + 0.12, 0.06, 0, 0, h + 0.03, material="White")


def whiteboard(ls, w=2.6, h=1.3):
    ls.box(w, 0.06, h, 0, 0, 0, material="White")
    ls.box(w + 0.08, 0.03, h + 0.08, 0, 0.03, 0, material="DarkGrey",
           collide=False)


def screen(ls, w=1.6, h=0.95):
    ls.box(w, 0.08, h, 0, 0, 0, material="FlatBlack")


def monitor(ls, z=0.75):
    """Desk monitor.  The screen is wide along x and thin along y, so its face
    is parallel to the long edge of the desk and points at the chair, which
    sits on the -y side."""
    ls.box(0.5, 0.06, 0.32, 0, 0, z + 0.28, material="FlatBlack")   # screen
    ls.box(0.18, 0.18, 0.02, 0, 0, z + 0.06, material="DarkGrey")   # foot
    ls.box(0.05, 0.05, 0.12, 0, 0, z + 0.12, material="DarkGrey")   # stem


def server_rack(ls):
    ls.box(0.8, 1.0, 2.0, 0, 0, 1.0, material="FlatBlack")
    for k in range(5):
        ls.box(0.02, 0.9, 0.06, -0.41, 0, 0.35 + k * 0.35, material="BlueGlow",
               collide=False)


def vending(ls, colour="Red"):
    ls.box(0.9, 0.75, 1.9, 0, 0, 0.95, material=colour)
    ls.box(0.02, 0.6, 1.2, -0.46, 0, 1.15, material="BlackTransparent",
           collide=False)


def forklift(ls):
    ls.box(1.4, 0.9, 0.8, 0, 0, 0.45, material="ZincYellow")     # body
    ls.box(0.9, 0.7, 0.06, 0.95, 0, 0.1, material="DarkGrey")    # forks
    ls.box(0.12, 0.8, 1.9, 0.55, 0, 0.95, material="DarkGrey")   # mast
    ls.box(0.7, 0.8, 0.06, -0.2, 0, 1.7, material="ZincYellow")  # roof cage
    for sx in (-1, 1):
        for sy in (-1, 1):
            ls.cylinder(0.18, 0.16, sx * 0.5, sy * 0.42, 0.18,
                        roll=math.pi / 2, material="FlatBlack")


def robot_arm(ls):
    ls.cylinder(0.28, 0.25, 0, 0, 0.125, material="DarkGrey")
    ls.cylinder(0.16, 0.85, 0, 0, 0.55, material="ZincYellow")
    ls.box(0.75, 0.16, 0.16, 0.3, 0, 1.05, material="ZincYellow")
    ls.box(0.12, 0.12, 0.28, 0.62, 0, 0.9, material="FlatBlack")


def fence(ls, w=2.0, h=1.2):
    ls.box(w, 0.04, h, 0, 0, h / 2, material="GreyTransparent")
    for sx in (-1, 1):
        ls.box(0.08, 0.08, h, sx * w / 2, 0, h / 2, material="ZincYellow")


def cone(ls):
    ls.cylinder(0.20, 0.03, 0, 0, 0.015, material="FlatBlack")
    ls.cylinder(0.11, 0.55, 0, 0, 0.30, material="Orange")


def bin_(ls):
    ls.cylinder(0.25, 0.8, 0, 0, 0.4, material="Turquoise")


def pallet(ls):
    ls.box(1.2, 1.0, 0.14, 0, 0, 0.07, material="WoodPallet")


# ----------------------------------------------------------------------------
# World assembly
# ----------------------------------------------------------------------------
def build():
    w = World()

    # ---- floor (visual only; the ground_plane model provides the collision)
    floor = LinkSet()
    floor.box(2 * HALF, 2 * HALF, 0.02, 0, 0, 0.01, material="Grey",
              collide=False)
    # corridor runner, purely cosmetic, makes the layout readable on camera
    floor.box(2 * HALF, 2 * CORR, 0.005, 0, 0, 0.02, material="SkyBlue",
              collide=False)
    w.add("floor", floor)

    # ---- outer shell
    add_wall_x(w, "wall_north", HALF, -HALF, HALF, material="Bricks")
    add_wall_x(w, "wall_south", -HALF, -HALF, HALF, material="Bricks")
    add_wall_y(w, "wall_west", -HALF, -HALF, HALF, material="Bricks")
    add_wall_y(w, "wall_east", HALF, -HALF, HALF, material="Bricks")

    # ---- corridor walls with doorways
    add_wall_x(w, "corridor_wall_north", CORR, -HALF, HALF,
               gaps=[(-11.0, DOOR_W), (0.0, DOOR_W), (9.0, DOOR_W)])
    add_wall_x(w, "corridor_wall_south", -CORR, -HALF, HALF,
               gaps=[(-11.0, DOOR_W), (0.0, DOOR_W), (9.0, DOOR_W)])

    # ---- room dividers
    add_wall_y(w, "divider_nw", -5.0, CORR, HALF)
    add_wall_y(w, "divider_ne", 5.0, CORR, HALF)
    add_wall_y(w, "divider_sw", -5.0, -HALF, -CORR)
    add_wall_y(w, "divider_se", 5.0, -HALF, -CORR)

    # ---- server room carved out of the reception corner
    add_wall_x(w, "server_wall", 9.0, -HALF, -11.0, gaps=[(-13.0, 1.4)])
    add_wall_y(w, "server_wall_side", -11.0, 9.0, HALF)

    # ========================= ROOM 1 : RECEPTION =========================
    ls = LinkSet(); counter(ls, w=3.2, d=0.8, h=1.1)
    w.add("reception_desk", ls, -13.0, 3.6, yaw=0.0)

    ls = LinkSet(); chair(ls, seat="Green")
    w.add("reception_chair", ls, -13.0, 4.5, yaw=math.pi)

    ls = LinkSet(); person(ls, shirt="Green")
    w.add("person_receptionist", ls, -13.0, 4.95, yaw=-math.pi / 2)

    ls = LinkSet(); sofa(ls, w=2.2, colour="Purple")
    w.add("lounge_sofa_a", ls, -7.6, 5.2, yaw=math.pi / 2)
    ls = LinkSet(); sofa(ls, w=2.2, colour="Purple")
    w.add("lounge_sofa_b", ls, -7.6, 8.0, yaw=math.pi / 2)
    ls = LinkSet(); sofa(ls, w=1.6, colour="Indigo")
    w.add("lounge_sofa_c", ls, -10.2, 6.6, yaw=0.0)

    ls = LinkSet(); desk(ls, w=1.2, d=0.7, h=0.45, top="FlatBlack")
    w.add("lounge_coffee_table", ls, -8.7, 6.6)

    for i, (px, py, sh) in enumerate([(-8.7, 4.4, "RedBright"),
                                      (-9.9, 8.4, "Yellow")]):
        ls = LinkSet(); person(ls, shirt=sh)
        w.add(f"person_lounge_{i}", ls, px, py, yaw=1.2 * i)

    for i, (px, py) in enumerate([(-14.4, 7.9), (-5.7, 3.0), (-5.7, 8.6),
                                  (-14.4, 2.6)]):
        ls = LinkSet(); plant(ls)
        w.add(f"plant_reception_{i}", ls, px, py)

    # Kept clear of the doorway at x = -11
    ls = LinkSet(); whiteboard(ls, w=2.0, h=0.9)
    w.add("reception_sign", ls, -13.2, 1.63, z=1.8)

    # ========================= ROOM 2 : SERVER ROOM =======================
    for i in range(4):
        ls = LinkSet(); server_rack(ls)
        w.add(f"server_rack_{i}", ls, -14.2, 10.4 + i * 1.2, yaw=0.0)
    for i in range(2):
        ls = LinkSet(); server_rack(ls)
        w.add(f"server_rack_b_{i}", ls, -11.9, 11.0 + i * 1.4, yaw=math.pi)
    ls = LinkSet(); cabinet(ls, w=1.0, d=0.6, h=1.6, colour="Turquoise")
    w.add("server_ups", ls, -13.0, 14.4)

    # ========================= ROOM 3 : OPEN-PLAN OFFICE ==================
    shirts = ["Blue", "RedBright", "Yellow", "Turquoise", "White", "Purple"]
    k = 0
    for row, dy in enumerate([5.0, 9.0, 12.6]):
        for col, dx in enumerate([-3.2, 0.0, 3.2]):
            ls = LinkSet()
            desk(ls, w=1.7, d=0.85)
            monitor(ls, z=0.75)
            w.add(f"office_desk_{k}", ls, dx, dy, yaw=0.0)

            ls = LinkSet(); chair(ls, seat="DarkGrey")
            w.add(f"office_chair_{k}", ls, dx, dy - 0.85, yaw=0.0)
            k += 1

    # Staff stand just behind their chair (chair sits at desk_y - 0.85)
    for i, (px, desk_y, sh) in enumerate([(-3.2, 5.0, shirts[0]),
                                          (0.0, 9.0, shirts[1]),
                                          (3.2, 12.6, shirts[2])]):
        ls = LinkSet(); person(ls, shirt=sh)
        w.add(f"person_office_{i}", ls, px, desk_y - 1.35, yaw=0.0)

    for i, px in enumerate([-4.2, -2.6, -1.0]):
        ls = LinkSet(); cabinet(ls)
        w.add(f"office_cabinet_{i}", ls, px, 14.4, yaw=math.pi)

    ls = LinkSet(); plant(ls)
    w.add("plant_office_a", ls, 4.2, 13.9)
    ls = LinkSet(); plant(ls)
    w.add("plant_office_b", ls, 4.2, 2.7)
    ls = LinkSet(); bin_(ls)
    w.add("office_bin", ls, -4.4, 2.7)

    # ========================= ROOM 4 : MEETING ROOM ======================
    ls = LinkSet(); desk(ls, w=4.2, d=1.7, h=0.76, top="Wood")
    w.add("meeting_table", ls, 10.0, 7.5, yaw=0.0)

    seat_id = 0
    for dx in (-1.4, 0.0, 1.4):
        for side in (-1, 1):
            ls = LinkSet(); chair(ls, seat="RedBright")
            w.add(f"meeting_chair_{seat_id}", ls,
                  10.0 + dx, 7.5 + side * 1.35,
                  yaw=0.0 if side < 0 else math.pi)
            seat_id += 1
    for dy in (-0.0,):
        for sx in (-1, 1):
            ls = LinkSet(); chair(ls, seat="RedBright")
            w.add(f"meeting_chair_{seat_id}", ls, 10.0 + sx * 2.6, 7.5 + dy,
                  yaw=math.pi / 2 if sx < 0 else -math.pi / 2)
            seat_id += 1

    ls = LinkSet(); whiteboard(ls, w=2.8, h=1.3)
    w.add("meeting_whiteboard", ls, 10.0, 14.85, z=1.6, yaw=0.0)
    ls = LinkSet(); screen(ls)
    w.add("meeting_screen", ls, 14.85, 7.5, z=1.7, yaw=math.pi / 2)
    ls = LinkSet(); cabinet(ls, w=2.0, d=0.5, h=0.9, colour="Wood")
    w.add("meeting_sideboard", ls, 6.0, 7.5, yaw=math.pi / 2)

    for i, (px, py, sh) in enumerate([(10.0, 13.6, "White"),
                                      (11.6, 13.2, "Blue")]):
        ls = LinkSet(); person(ls, shirt=sh)
        w.add(f"person_meeting_{i}", ls, px, py, yaw=-math.pi / 2)

    ls = LinkSet(); plant(ls)
    w.add("plant_meeting", ls, 6.0, 13.8)

    # ========================= ROOM 5 : WAREHOUSE =========================
    # Racks span x = -13.5 .. -7.5, leaving a 1.4 m service lane against the
    # west wall and a 2.4 m picking aisle on the east side.
    for i, ry in enumerate([-4.8, -7.8, -10.8, -13.8]):
        ls = LinkSet(); rack(ls, w=6.0, d=1.1, h=2.3, levels=3)
        w.add(f"warehouse_rack_{i}", ls, -10.5, ry, yaw=0.0)

    boxes = [(-6.6, -6.0), (-6.6, -7.1), (-6.6, -12.0), (-6.6, -13.1),
             (-14.4, -2.9), (-13.3, -2.9)]
    for i, (px, py) in enumerate(boxes):
        ls = LinkSet(); crate(ls, s=0.75)
        w.add(f"warehouse_crate_{i}", ls, px, py, yaw=0.15 * i)

    for i, (px, py) in enumerate([(-8.6, -3.6), (-7.3, -3.6)]):
        ls = LinkSet(); pallet(ls)
        w.add(f"warehouse_pallet_{i}", ls, px, py)

    ls = LinkSet(); forklift(ls)
    w.add("warehouse_forklift", ls, -11.0, -3.4, yaw=0.0)

    ls = LinkSet(); person(ls, shirt="ZincYellow")
    w.add("person_warehouse_a", ls, -12.2, -3.0, yaw=0.4)
    ls = LinkSet(); person(ls, shirt="ZincYellow")
    w.add("person_warehouse_b", ls, -10.0, -6.3, yaw=math.pi)

    ls = LinkSet(); cabinet(ls, w=1.2, d=0.6, h=1.8, colour="RedBright")
    w.add("warehouse_tool_cabinet", ls, -14.2, -14.2)

    # ========================= ROOM 6 : CAFETERIA =========================
    # Rows are 3.6 m apart so a person can stand between the chair backs
    t = 0
    for ty in (-5.0, -8.6, -12.2):
        for tx in (-3.0, 0.2, 3.4):
            ls = LinkSet(); round_table(ls, radius=0.75)
            w.add(f"cafe_table_{t}", ls, tx, ty)
            for s, (cx, cy, cyaw) in enumerate(
                    [(0.0, 1.15, math.pi), (0.0, -1.15, 0.0)]):
                ls = LinkSet(); chair(ls, seat="Turquoise")
                w.add(f"cafe_chair_{t}_{s}", ls, tx + cx, ty + cy, yaw=cyaw)
            t += 1

    ls = LinkSet(); counter(ls, w=6.0, d=0.9, h=1.05)
    w.add("cafe_counter", ls, 0.0, -14.2, yaw=0.0)

    for i, (px, colour) in enumerate([(-4.2, "Red"), (4.2, "Blue")]):
        ls = LinkSet(); vending(ls, colour=colour)
        w.add(f"cafe_vending_{i}", ls, px, -14.3, yaw=0.0)

    # Diners stand just outside the chairs (chairs sit at table_y +- 1.15)
    for i, (px, py, sh, yw) in enumerate([(1.8, -13.4, "White", -math.pi / 2),
                                          (-3.0, -6.80, "Orange", math.pi),
                                          (3.4, -10.40, "Purple", 0.0)]):
        ls = LinkSet(); person(ls, shirt=sh)
        w.add(f"person_cafe_{i}", ls, px, py, yaw=yw)

    ls = LinkSet(); plant(ls)
    w.add("plant_cafe_a", ls, -4.3, -3.0)
    ls = LinkSet(); plant(ls)
    w.add("plant_cafe_b", ls, 4.3, -3.0)
    ls = LinkSet(); bin_(ls)
    w.add("cafe_bin", ls, 4.55, -4.05)

    # ========================= ROOM 7 : ROBOTICS LAB ======================
    for i, by in enumerate([-4.4, -7.2, -10.0]):
        ls = LinkSet(); desk(ls, w=2.4, d=0.9, h=0.85, top="Grey")
        w.add(f"lab_bench_{i}", ls, 13.8, by, yaw=math.pi / 2)

    ls = LinkSet(); robot_arm(ls)
    w.add("lab_arm_a", ls, 8.5, -6.0, yaw=0.0)
    ls = LinkSet(); robot_arm(ls)
    w.add("lab_arm_b", ls, 8.5, -9.6, yaw=math.pi)

    # safety fence around the robot cell
    for i, (fx, fy, fyaw) in enumerate([(7.0, -7.8, math.pi / 2),
                                        (10.0, -7.8, math.pi / 2),
                                        (8.5, -4.4, 0.0)]):
        ls = LinkSet(); fence(ls, w=3.2, h=1.3)
        w.add(f"lab_fence_{i}", ls, fx, fy, yaw=fyaw)

    # obstacle course for the mobile robot
    for i, (px, py) in enumerate([(11.6, -11.4), (12.8, -12.4), (11.0, -13.2),
                                  (12.6, -10.6), (10.2, -12.0)]):
        ls = LinkSet(); cone(ls)
        w.add(f"lab_cone_{i}", ls, px, py)
    for i, (px, py) in enumerate([(6.6, -12.0), (6.6, -12.8), (7.4, -12.0)]):
        ls = LinkSet(); crate(ls, s=0.6, colour="ZincYellow")
        w.add(f"lab_crate_{i}", ls, px, py, yaw=0.4 * i)

    ls = LinkSet(); cabinet(ls, w=1.2, d=0.6, h=1.8, colour="Blue")
    w.add("lab_cabinet", ls, 5.9, -3.2, yaw=math.pi / 2)
    ls = LinkSet(); whiteboard(ls, w=2.4, h=1.2)
    w.add("lab_whiteboard", ls, 11.5, -14.85, z=1.6)

    for i, (px, py, sh, yw) in enumerate([(9.9, -5.2, "White", math.pi),
                                          (13.0, -6.4, "Blue", -math.pi / 2)]):
        ls = LinkSet(); person(ls, shirt=sh)
        w.add(f"person_lab_{i}", ls, px, py, yaw=yw)

    ls = LinkSet(); plant(ls)
    w.add("plant_lab", ls, 14.2, -2.6)

    # ========================= CORRIDOR ===================================
    for i, px in enumerate([-6.0, 3.0, 12.0]):
        ls = LinkSet(); plant(ls)
        w.add(f"plant_corridor_{i}", ls, px, 1.2)
    ls = LinkSet(); desk(ls, w=1.6, d=0.45, h=0.45, top="Wood")
    w.add("corridor_bench", ls, -2.5, -1.2)
    ls = LinkSet(); bin_(ls)
    w.add("corridor_bin", ls, 6.5, -1.2)
    # Stands to one side of the corridor so the 3.5 m through-route stays open
    ls = LinkSet(); person(ls, shirt="Orange")
    w.add("person_corridor", ls, 5.0, 1.05, yaw=math.pi)
    ls = LinkSet(); cabinet(ls, w=0.8, d=0.4, h=1.9, colour="DarkGrey")
    w.add("corridor_locker", ls, -8.5, -1.35)

    return w


HEADER = """<?xml version="1.0" ?>
<!-- ==========================================================
     30 m x 30 m indoor world: "Smart Building"
     7 rooms + central corridor, furnished with desks, chairs,
     tables, shelving, machines and people.
     Generated by tools/generate_world.py - edit that file, not
     this one, and re-run it to regenerate.
     ========================================================== -->
<sdf version="1.6">
  <world name="smart_building">

    <!-- Sun and ground are defined inline instead of pulled in with
         model:// so the world never touches the online model database. -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material><script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Residential</name>
          </script></material>
        </visual>
      </link>
    </model>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <shadows>false</shadows>
    </scene>

    <physics name="default_physics" default="true" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <gui>
      <camera name="user_camera">
        <pose>-20 -20 22 0 0.72 0.78</pose>
      </camera>
    </gui>

"""

FOOTER = """
  </world>
</sdf>
"""


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "worlds", "smart_building.world")
    world = build()
    with open(out, "w") as fh:
        fh.write(HEADER)
        fh.write("\n".join(world.models))
        fh.write(FOOTER)
    n_links = sum(m.count("<link name=") for m in world.models)
    print(f"wrote {out}: {len(world.models)} models, {n_links} links")


if __name__ == "__main__":
    main()
