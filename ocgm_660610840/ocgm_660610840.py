import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ตั้งค่า stdout รองรับภาษาไทยบน Windows Terminal
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass


# ==============================================================================
# พารามิเตอร์ระบบ (Configuration Parameters)
# ==============================================================================
DATA_FILE = "data.csv"       # ไฟล์ข้อมูล lidar
GRID_SIZE = 101              # ขนาดแผนที่ 101 x 101 เซลล์ (ดัชนี 0...100)

# ค่า log-odds ที่ใช้ปรับความเชื่อของแต่ละเซลล์ (Inverse sensor model)
L_OCC = 0.85                 # เซลล์ปลายลำแสง = มีสิ่งกีดขวาง (เพิ่มค่า)
L_FREE = -0.40               # เซลล์ที่ลำแสงผ่าน = ว่าง (ลดค่า)
L_LIMIT = 8.0                # จำกัดค่า log-odds ไม่ให้สูงหรือต่ำเกินไป
L_PRIOR = 0.0                # ค่าเริ่มต้น = ไม่ทราบสถานะ (p = 0.5)

ANIM_INTERVAL = 30           # หน่วงเวลาระหว่างเฟรม (มิลลิวินาที)

# ค่าที่ใช้ในโหมดขับหุ่นยนต์เอง (--drive)
DRIVE_STEP = 2.0             # ระยะที่หุ่นยนต์ขยับต่อการกดปุ่ม 1 ครั้ง (เซลล์)
DRIVE_TURN = np.deg2rad(12)  # มุมที่หุ่นยนต์หมุนต่อการกดปุ่ม 1 ครั้ง (เรเดียน)
DRIVE_MAX_RANGE = 120.0      # ระยะไกลสุดที่ lidar มองเห็น (เซลล์)
WORLD_THRESHOLD = 0.6        # ความน่าจะเป็นที่ถือว่าเซลล์นั้นเป็นกำแพงจริง


# ==============================================================================
# อัลกอริทึม Bresenham's Line Algorithm
# คัดลอกมาจากงานก่อนหน้า bresenham_laser.py (อ้างอิง Lecture 09b หน้า 11, 12)
# ==============================================================================

def plotLineLow(x0, y0, x1, y1):
    """กรณี |slope| < 1 : เดินตามแกน x ทีละช่อง แล้วก้าวแกน y เมื่อ D > 0"""
    dx = x1 - x0
    dy = y1 - y0
    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy

    D = (2 * dy) - dx
    y = y0

    points = []
    for x in range(x0, x1 + 1):
        points.append((x, y))
        if D > 0:
            y = y + yi
            D = D + (2 * (dy - dx))
        else:
            D = D + 2 * dy

    return points


def plotLineHigh(x0, y0, x1, y1):
    """กรณี |slope| >= 1 : เดินตามแกน y ทีละช่อง แล้วก้าวแกน x เมื่อ D > 0"""
    dx = x1 - x0
    dy = y1 - y0
    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx

    D = (2 * dx) - dy
    x = x0

    points = []
    for y in range(y0, y1 + 1):
        points.append((x, y))
        if D > 0:
            x = x + xi
            D = D + (2 * (dx - dy))
        else:
            D = D + 2 * dx

    return points


def plotLine(x0, y0, x1, y1):
    """
    รวมทุกกรณีของความชัน และเรียงลำดับจุดจาก (x0, y0) ไปยัง (x1, y1) เสมอ
    เมื่อสลับพิกัดเพื่อเรียก plotLineLow / plotLineHigh ต้อง reverse ลิสต์กลับ
    """
    if abs(y1 - y0) < abs(x1 - x0):
        if x0 > x1:
            points = plotLineLow(x1, y1, x0, y0)
            points.reverse()
            return points
        else:
            return plotLineLow(x0, y0, x1, y1)
    else:
        if y0 > y1:
            points = plotLineHigh(x1, y1, x0, y0)
            points.reverse()
            return points
        else:
            return plotLineHigh(x0, y0, x1, y1)


# ==============================================================================
# ส่วนอ่านข้อมูลและแปลงพิกัด
# ==============================================================================

def load_scans(path):
    """
    อ่านไฟล์ CSV แล้วแยกออกเป็นตำแหน่งหุ่นยนต์ (pose) และข้อมูลลำแสง (z, phi)

    คืนค่า
        poses : array รูปร่าง (n, 3)        คือ x, y, theta ของแต่ละแถว
        z     : array รูปร่าง (n, n_beams)  คือ ระยะทางที่วัดได้
        phi   : array รูปร่าง (n, n_beams)  คือ มุมลำแสงเทียบตัวหุ่นยนต์
    """
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    poses = data[:, 0:3]
    z = data[:, 3::2]      # คอลัมน์ 4, 6, 8, ... (ดัชนี 3, 5, 7, ...)
    phi = data[:, 4::2]    # คอลัมน์ 5, 7, 9, ... (ดัชนี 4, 6, 8, ...)
    return poses, z, phi


def beam_endpoint(x, y, theta, z_k, phi_k):
    """
    หาจุดปลายลำแสงในกรอบพิกัดโลก
    มุมของลำแสงในกรอบโลก = theta (ทิศหุ่นยนต์) + phi_k (มุมลำแสงเทียบหุ่นยนต์)
    """
    angle = theta + phi_k
    ex = x + z_k * np.cos(angle)
    ey = y + z_k * np.sin(angle)
    return ex, ey


def to_cell(x, y, grid_size=GRID_SIZE):
    """แปลงพิกัดจริงเป็นดัชนีเซลล์จำนวนเต็ม และบังคับให้อยู่ในขอบเขตแผนที่"""
    i = int(round(x))
    j = int(round(y))
    i = max(0, min(grid_size - 1, i))
    j = max(0, min(grid_size - 1, j))
    return i, j


# ==============================================================================
# คลาสสร้าง Occupancy Grid Map
# ==============================================================================

class OccupancyGridMap:
    """
    เก็บแผนที่ในรูป log-odds แล้วแปลงเป็นความน่าจะเป็นเมื่อต้องการแสดงผล
        log_odds > 0  -> น่าจะมีสิ่งกีดขวาง (สีดำ)
        log_odds < 0  -> น่าจะเป็นที่ว่าง (สีขาว)
        log_odds = 0  -> ยังไม่ทราบ (สีเทา)
    """

    def __init__(self, grid_size=GRID_SIZE):
        self.grid_size = grid_size
        self.log_odds = np.full((grid_size, grid_size), L_PRIOR, dtype=float)

    def update_cell(self, i, j, delta):
        """ปรับค่า log-odds ของเซลล์หนึ่งเซลล์ พร้อมจำกัดค่าไม่ให้เกินขอบเขต"""
        value = self.log_odds[j, i] + delta      # แถว = y, คอลัมน์ = x
        self.log_odds[j, i] = max(-L_LIMIT, min(L_LIMIT, value))

    def integrate_beam(self, x, y, theta, z_k, phi_k):
        """
        นำลำแสง 1 เส้นเข้ารวมกับแผนที่
        1. หาจุดปลายลำแสงจาก z และ phi
        2. ใช้ Bresenham หาเซลล์ทั้งหมดบนเส้นจากหุ่นยนต์ไปยังจุดปลาย
        3. เซลล์ระหว่างทางเป็นที่ว่าง เซลล์สุดท้ายเป็นสิ่งกีดขวาง
        """
        if np.isnan(z_k) or np.isnan(phi_k):
            return []                            # ลำแสงไม่พบสิ่งกีดขวาง จึงข้าม

        ex, ey = beam_endpoint(x, y, theta, z_k, phi_k)
        i0, j0 = to_cell(x, y, self.grid_size)
        i1, j1 = to_cell(ex, ey, self.grid_size)

        cells = plotLine(i0, j0, i1, j1)
        for (ci, cj) in cells[:-1]:
            self.update_cell(ci, cj, L_FREE)
        ci, cj = cells[-1]
        self.update_cell(ci, cj, L_OCC)
        return cells

    def integrate_scan(self, pose, z_row, phi_row):
        """นำข้อมูลลำแสงทุกเส้นของตำแหน่งหนึ่งเข้ารวมกับแผนที่"""
        x, y, theta = pose
        beams = []
        for z_k, phi_k in zip(z_row, phi_row):
            cells = self.integrate_beam(x, y, theta, z_k, phi_k)
            if cells:
                beams.append(cells)
        return beams

    def probability(self):
        """แปลง log-odds เป็นความน่าจะเป็นที่เซลล์จะมีสิ่งกีดขวาง (0.0 ถึง 1.0)"""
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))


# ==============================================================================
# ส่วนแสดงผล
# ==============================================================================

def make_figure(grid_map, title):
    """สร้างหน้าต่างแสดงแผนที่ พร้อมคืนค่า objects ที่ต้องใช้อัปเดตภาพ"""
    fig, ax = plt.subplots(figsize=(7.5, 7.5))

    # ใช้ colormap สีเทา แบบกลับด้าน: p = 0 เป็นสีขาว, p = 1 เป็นสีดำ
    img = ax.imshow(
        grid_map.probability(),
        cmap="gray_r",
        vmin=0.0,
        vmax=1.0,
        origin="lower",
        extent=[0, grid_map.grid_size, 0, grid_map.grid_size],
        interpolation="nearest",
    )

    path_line, = ax.plot([], [], "-", color="#1f77b4", linewidth=1.2, label="robot path")
    beam_line, = ax.plot([], [], "-", color="red", linewidth=0.6, alpha=0.6, label="lidar beams")
    robot_dot, = ax.plot([], [], "o", color="#ff7f0e", markersize=7, label="robot pose")

    ax.set_xlim(0, grid_map.grid_size)
    ax.set_ylim(0, grid_map.grid_size)
    ax.set_xlabel("x (cell)")
    ax.set_ylabel("y (cell)")
    ax.set_title(title)
    ax.grid(False)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    return fig, ax, img, path_line, beam_line, robot_dot


def run_animation(grid_map, poses, z, phi, save_path=None):
    """แสดงภาพเคลื่อนไหว: เดินทีละแถวของข้อมูล แล้ววาดแผนที่ที่โตขึ้นเรื่อย ๆ"""
    fig, ax, img, path_line, beam_line, robot_dot = make_figure(
        grid_map, "Occupancy Grid Map (building...)"
    )
    path_x, path_y = [], []

    def update(k):
        beams = grid_map.integrate_scan(poses[k], z[k], phi[k])

        # เก็บเส้นทางที่หุ่นยนต์เคลื่อนผ่าน
        path_x.append(poses[k, 0])
        path_y.append(poses[k, 1])

        # วาดลำแสงของตำแหน่งปัจจุบัน โดยคั่นแต่ละเส้นด้วย NaN
        bx, by = [], []
        for cells in beams:
            bx += [poses[k, 0], cells[-1][0] + 0.5, np.nan]
            by += [poses[k, 1], cells[-1][1] + 0.5, np.nan]

        img.set_data(grid_map.probability())
        path_line.set_data(path_x, path_y)
        beam_line.set_data(bx, by)
        robot_dot.set_data([poses[k, 0]], [poses[k, 1]])
        ax.set_title(f"Occupancy Grid Map  —  scan {k + 1}/{len(poses)}")
        return img, path_line, beam_line, robot_dot

    anim = FuncAnimation(
        fig, update, frames=len(poses),
        interval=ANIM_INTERVAL, blit=False, repeat=False,
    )

    if save_path:
        # วาดทุกเฟรมให้จบก่อน แล้วจึงบันทึกภาพสุดท้าย
        for k in range(len(poses)):
            update(k)
        beam_line.set_data([], [])
        fig.savefig(save_path, dpi=150)
        print(f"บันทึกแผนที่แล้วที่ {save_path}")

    plt.show()
    return anim


def show_final(grid_map, poses, z, phi, save_path=None):
    """คำนวณทุกแถวให้จบก่อน แล้วค่อยแสดงแผนที่สุดท้ายครั้งเดียว"""
    for k in range(len(poses)):
        grid_map.integrate_scan(poses[k], z[k], phi[k])

    fig, ax, img, path_line, beam_line, robot_dot = make_figure(
        grid_map, "Occupancy Grid Map (final)"
    )
    img.set_data(grid_map.probability())
    path_line.set_data(poses[:, 0], poses[:, 1])
    robot_dot.set_data([poses[-1, 0]], [poses[-1, 1]])

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"บันทึกแผนที่แล้วที่ {save_path}")

    plt.show()


# ==============================================================================
# โหมดขับหุ่นยนต์เอง (Manual drive mode)
# แนวคิด: ใช้ข้อมูลใน data.csv สร้าง "โลกจริง" ขึ้นมาก่อน จากนั้นให้ผู้ใช้ขับ
# หุ่นยนต์เดินในโลกนั้น แล้วยิง lidar จำลองเพื่อสร้างแผนที่ใหม่ขึ้นมาเอง
# ==============================================================================

def build_world(poses, z, phi, grid_size=GRID_SIZE):
    """
    สร้างกำแพงจริงของโลกจากข้อมูลทั้งไฟล์
    คืนค่าเป็น array แบบ bool : True = เซลล์นั้นเป็นกำแพง
    """
    reference = OccupancyGridMap(grid_size)
    for k in range(len(poses)):
        reference.integrate_scan(poses[k], z[k], phi[k])
    return reference.probability() > WORLD_THRESHOLD


def cast_ray(world, x, y, angle, max_range=DRIVE_MAX_RANGE):
    """
    ยิงลำแสงจำลอง 1 เส้นจากจุด (x, y) ไปตามมุม angle แล้วหาว่าชนกำแพงที่ระยะเท่าใด
    ใช้ Bresenham เดินไปทีละเซลล์จนกว่าจะเจอกำแพง

    คืนค่า ระยะทางที่วัดได้ (เซลล์) หรือ None เมื่อไม่ชนอะไรเลย
    """
    grid_size = world.shape[0]
    ex = x + max_range * np.cos(angle)
    ey = y + max_range * np.sin(angle)

    i0, j0 = to_cell(x, y, grid_size)
    i1, j1 = to_cell(ex, ey, grid_size)

    for (ci, cj) in plotLine(i0, j0, i1, j1)[1:]:
        if world[cj, ci]:
            return float(np.hypot(ci - x, cj - y))
    return None


class ManualDriveSession:
    """
    หน้าต่างที่ให้ผู้ใช้ขับหุ่นยนต์เอง แล้วสร้าง occupancy grid map ไปพร้อมกัน

    ปุ่มควบคุม
        ลูกศรขึ้น / W      เดินหน้า
        ลูกศรลง / S        ถอยหลัง
        ลูกศรซ้าย / A      หมุนซ้าย
        ลูกศรขวา / D       หมุนขวา
        R                  ล้างแผนที่ที่สร้างไว้ แล้วเริ่มใหม่
        ESC / Q            ออกจากโปรแกรม
    """

    def __init__(self, world, beam_angles, start_pose):
        self.world = world
        self.beam_angles = beam_angles
        self.grid_map = OccupancyGridMap(world.shape[0])
        self.x, self.y, self.theta = start_pose
        self.path_x = [self.x]
        self.path_y = [self.y]

        (self.fig, self.ax, self.img, self.path_line,
         self.beam_line, self.robot_dot) = make_figure(
            self.grid_map, "Manual drive  —  arrows / WASD to move, R reset, ESC quit"
        )
        self.head_line, = self.ax.plot([], [], "-", color="#ff7f0e", linewidth=2.0)

        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.scan()
        self.redraw()

    # ---------------------------------------------------------------- sensing
    def scan(self):
        """ยิง lidar ทุกลำแสงจากตำแหน่งปัจจุบัน แล้วรวมผลเข้าแผนที่"""
        beams = []
        for phi_k in self.beam_angles:
            z_k = cast_ray(self.world, self.x, self.y, self.theta + phi_k)
            if z_k is None:
                continue                         # ไม่ชนอะไร จึงไม่อัปเดตแผนที่
            cells = self.grid_map.integrate_beam(self.x, self.y, self.theta, z_k, phi_k)
            if cells:
                beams.append(cells)
        self.beams = beams

    # ---------------------------------------------------------------- motion
    def is_free(self, x, y):
        """ตรวจว่าตำแหน่งใหม่อยู่ในแผนที่ และไม่ทับกำแพงจริง"""
        grid_size = self.world.shape[0]
        if not (0 <= x <= grid_size - 1 and 0 <= y <= grid_size - 1):
            return False
        i, j = to_cell(x, y, grid_size)
        return not self.world[j, i]

    def move(self, distance):
        """เดินหน้าหรือถอยหลังตามทิศที่หุ่นยนต์หันอยู่ ถ้าชนกำแพงจะไม่ขยับ"""
        nx = self.x + distance * np.cos(self.theta)
        ny = self.y + distance * np.sin(self.theta)
        if not self.is_free(nx, ny):
            return False
        self.x, self.y = nx, ny
        self.path_x.append(nx)
        self.path_y.append(ny)
        return True

    def turn(self, angle):
        """หมุนตัวหุ่นยนต์ และคุมมุมให้อยู่ในช่วง -pi ถึง pi"""
        self.theta = (self.theta + angle + np.pi) % (2 * np.pi) - np.pi

    def reset_map(self):
        """ล้างแผนที่ที่สร้างไว้ทั้งหมด แต่หุ่นยนต์ยังอยู่ที่เดิม"""
        self.grid_map = OccupancyGridMap(self.world.shape[0])
        self.path_x = [self.x]
        self.path_y = [self.y]

    # ---------------------------------------------------------------- display
    def redraw(self):
        """วาดแผนที่ ลำแสง เส้นทาง และทิศของหุ่นยนต์ใหม่ทั้งหมด"""
        bx, by = [], []
        for cells in self.beams:
            bx += [self.x, cells[-1][0], np.nan]
            by += [self.y, cells[-1][1], np.nan]

        head_len = 4.0
        self.img.set_data(self.grid_map.probability())
        self.beam_line.set_data(bx, by)
        self.path_line.set_data(self.path_x, self.path_y)
        self.robot_dot.set_data([self.x], [self.y])
        self.head_line.set_data(
            [self.x, self.x + head_len * np.cos(self.theta)],
            [self.y, self.y + head_len * np.sin(self.theta)],
        )
        known = int((self.grid_map.log_odds != L_PRIOR).sum())
        total = self.grid_map.grid_size ** 2
        self.ax.set_title(
            f"Manual drive  —  pose ({self.x:.1f}, {self.y:.1f}, "
            f"{np.rad2deg(self.theta):.0f}°)   explored {100.0 * known / total:.1f}%"
        )
        self.fig.canvas.draw_idle()

    # ---------------------------------------------------------------- control
    def on_key(self, event):
        """รับปุ่มที่ผู้ใช้กด แล้วสั่งหุ่นยนต์ทำงานตามปุ่มนั้น"""
        key = (event.key or "").lower()

        if key in ("escape", "q"):
            plt.close(self.fig)
            return
        elif key in ("up", "w"):
            self.move(DRIVE_STEP)
        elif key in ("down", "s"):
            self.move(-DRIVE_STEP)
        elif key in ("left", "a"):
            self.turn(DRIVE_TURN)
        elif key in ("right", "d"):
            self.turn(-DRIVE_TURN)
        elif key == "r":
            self.reset_map()
        else:
            return                               # ปุ่มอื่นไม่ทำอะไร

        self.scan()
        self.redraw()


def run_manual_drive(poses, z, phi, save_path=None):
    """เตรียมโลกจริงและมุมลำแสง จากนั้นเปิดหน้าต่างให้ผู้ใช้ขับหุ่นยนต์เอง"""
    world = build_world(poses, z, phi)

    # ใช้ชุดมุมลำแสงเดียวกับในไฟล์ข้อมูล เลือกแถวที่มีค่าครบทุกลำแสง
    beam_angles = None
    for row in phi:
        if not np.isnan(row).any():
            beam_angles = row
            break
    if beam_angles is None:
        beam_angles = np.linspace(-np.pi / 2, np.pi / 2, phi.shape[1])

    session = ManualDriveSession(world, beam_angles, poses[0])

    print("ควบคุมหุ่นยนต์: ลูกศร หรือ W A S D = เดิน/หมุน, R = ล้างแผนที่, ESC = ออก")
    plt.show()

    if save_path:
        session.fig.savefig(save_path, dpi=150)
        print(f"บันทึกแผนที่แล้วที่ {save_path}")
    return session


# ==============================================================================
# main
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(description="สร้าง Occupancy Grid Map จากข้อมูล lidar")
    parser.add_argument("--data", default=DATA_FILE, help="ตำแหน่งไฟล์ข้อมูล CSV")
    parser.add_argument("--no-anim", action="store_true", help="ไม่ต้องแสดงภาพเคลื่อนไหว")
    parser.add_argument("--save", default=None, help="บันทึกแผนที่สุดท้ายเป็นไฟล์รูป")
    parser.add_argument("--drive", action="store_true",
                        help="ขับหุ่นยนต์เองด้วยปุ่มลูกศร หรือ W A S D")
    args = parser.parse_args()

    poses, z, phi = load_scans(args.data)
    print(f"อ่านข้อมูลได้ {len(poses)} แถว, ลำแสง {z.shape[1]} เส้นต่อแถว")
    print(f"ขอบเขตตำแหน่งหุ่นยนต์ x = {poses[:, 0].min():.1f}...{poses[:, 0].max():.1f}, "
          f"y = {poses[:, 1].min():.1f}...{poses[:, 1].max():.1f}")

    if args.drive:
        run_manual_drive(poses, z, phi, args.save)
        return

    grid_map = OccupancyGridMap(GRID_SIZE)

    if args.no_anim:
        show_final(grid_map, poses, z, phi, args.save)
    else:
        run_animation(grid_map, poses, z, phi, args.save)


if __name__ == "__main__":
    main()
