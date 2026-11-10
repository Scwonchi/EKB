"""Имитация подъезда без ровера и ROS.

Сцена как на площадке: красное прямоугольное окно высоко на двух чёрных стойках с Т-образными
опорами, под рамкой чёрная табличка; вокруг красные помехи — сплошной ящик, мяч и красная рамка
того же размера, стоящая на полу. Камера — на выбор:

* ``webcam`` — веб-камера на второй Raspberry Pi: обычный объектив (с небольшой бочкой, которую
  нода не знает), на мачте, наклонена вверх;
* ``fisheye`` — штатная камера ровера: «рыбий глаз», низко, чуть наклонена.

Регулятор ведёт ровер, кинематика меканума — с шумом и задержкой кадра.

    python3 -m rover_window_approach.sim                       # 27 попыток, веб-камера
    python3 -m rover_window_approach.sim --camera fisheye      # штатная камера ровера
    python3 -m rover_window_approach.sim --lens-error          # угол обзора и наклон камеры заданы с ошибкой
    python3 -m rover_window_approach.sim --show paths.png --frame frame.png
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field, replace
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .lens import LensParams, Rectifier
from .window_core import (DONE, ApproachParams, CameraModel, WindowApproach, WindowGeometry, detect_candidates,
                          draw_debug, find_window)


# ---------------------------------------------------------------- сцена


@dataclass
class Scene:
    window_x: float = 0.0          # центр окна (в плане) в мире, м
    window_y: float = 0.0
    window_yaw: float = 0.0        # куда смотрит лицевая сторона окна (нормаль), рад
    width: float = 0.85            # внешний размер рамки (окно на площадке ~85 × 70 см)
    height: float = 0.70
    frame_w: float = 0.05          # ширина красной планки
    bottom: float = 0.85           # низ рамки над полом (центр на 1,2 м)
    leg_spacing: float = 0.80      # между стойками по центрам
    leg_w: float = 0.03            # толщина стойки
    distractors: bool = True


@dataclass
class Lens:
    """Настоящая камера в имитации: «рыбий глаз» (модель cv2.fisheye) или обычный объектив."""
    kind: str = "fisheye"          # fisheye | pinhole
    width: int = 1280
    height: int = 1024
    circle_r: float = 500.0        # fisheye: радиус круга изображения, px
    fov_deg: float = 190.0         # fisheye: угол обзора по диаметру круга
    hfov_deg: float = 70.0         # pinhole: угол обзора по горизонтали
    D: Tuple[float, float, float, float] = (0.03, -0.01, 0.0, 0.0)   # pinhole: k1 — бочка (r·(1 + k1·r²))
    pitch_deg: float = 4.0         # камера смотрит вверх на столько градусов
    cam_x: float = 0.12            # камера относительно центра ровера, м
    cam_y: float = 0.0
    cam_h: float = 0.10            # высота объектива над полом

    @property
    def K(self) -> np.ndarray:
        if self.kind == "pinhole":
            f = (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)
        else:
            # f подобран так, чтобы край круга (θ = fov/2) с учётом D лёг на circle_r
            t = math.radians(self.fov_deg / 2)
            k1, k2, k3, k4 = self.D
            f = self.circle_r / (t * (1 + k1 * t ** 2 + k2 * t ** 4 + k3 * t ** 6 + k4 * t ** 8))
        return np.array([[f, 0, (self.width - 1) / 2], [0, f, (self.height - 1) / 2], [0, 0, 1]], np.float64)

    def project(self, pc: np.ndarray) -> np.ndarray:
        """Точки в осях камеры (x вправо, y вниз, z вперёд) → пиксели."""
        K = self.K
        x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]
        if self.kind == "pinhole":
            a, b = x / z, y / z
            k = 1 + self.D[0] * np.minimum(a * a + b * b, 1.2)    # за краем кадра — без бочки (иначе вывернет)
            return np.c_[K[0, 0] * a * k + K[0, 2], K[1, 1] * b * k + K[1, 2]]
        rho = np.hypot(x, y)
        th = np.arctan2(rho, z)
        k1, k2, k3, k4 = self.D
        td = th * (1 + k1 * th ** 2 + k2 * th ** 4 + k3 * th ** 6 + k4 * th ** 8)
        s = np.where(rho > 1e-9, td / np.maximum(rho, 1e-9), 0.0)
        return np.c_[K[0, 0] * s * x + K[0, 2], K[1, 1] * s * y + K[1, 2]]


FISHEYE = Lens()
# Веб-камера (как Logitech C920 в режиме 640×480): на мачте 45 см, наклон вверх 30°, слабая бочка
WEBCAM = Lens(kind="pinhole", width=640, height=480, hfov_deg=70.0, D=(-0.04, 0.0, 0.0, 0.0),
              pitch_deg=30.0, cam_x=0.05, cam_h=0.45)


def _rx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class Camera:
    """Рисует кадр камеры с того места, где стоит ровер."""

    def __init__(self, lens: Lens, seed: int = 0):
        self.lens = lens
        self.R_src = _rx(-lens.pitch_deg)                # ровные оси → оси наклонённой камеры
        h, w = lens.height, lens.width
        # Фон: для каждого пикселя — луч; ниже горизонта пол, выше стена, вне круга — чёрное
        K = lens.K
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        a, b = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
        if lens.kind == "pinhole":
            ray = np.stack([a, b, np.ones_like(a)], -1)
            self.outside = np.zeros((h, w), bool)
        else:
            td = np.hypot(a, b)
            th = td.copy()
            k1, k2, k3, k4 = lens.D
            for _ in range(8):                            # θ из θd (Ньютон)
                f = th * (1 + k1 * th ** 2 + k2 * th ** 4 + k3 * th ** 6 + k4 * th ** 8) - td
                df = 1 + 3 * k1 * th ** 2 + 5 * k2 * th ** 4 + 7 * k3 * th ** 6 + 9 * k4 * th ** 8
                th = th - f / df
            s = np.where(td > 1e-9, np.sin(th) / np.maximum(td, 1e-9), 0)
            ray = np.stack([a * s, b * s, np.cos(th)], -1)
            t = math.radians(lens.fov_deg / 2)
            self.outside = td > t * (1 + k1 * t ** 2 + k2 * t ** 4 + k3 * t ** 6 + k4 * t ** 8)
        level = ray @ self.R_src                           # = R_src^T · ray
        bg = np.empty((h, w, 3), np.uint8)
        bg[:] = (128, 132, 126)                            # стены
        bg[level[..., 1] > 0] = (88, 92, 98)               # пол
        bg[self.outside] = (6, 6, 6)
        self.bg = bg
        rng = np.random.default_rng(seed)
        self.noise = [rng.integers(-10, 11, (h, w, 3), dtype=np.int16) for _ in range(3)]
        self.k = 0

    def to_cam(self, robot, pts_world: np.ndarray) -> np.ndarray:
        rx, ry, ryaw = robot
        L = self.lens
        c, s = math.cos(ryaw), math.sin(ryaw)
        dx, dy = pts_world[:, 0] - rx, pts_world[:, 1] - ry
        fwd = c * dx + s * dy - L.cam_x
        left = -s * dx + c * dy - L.cam_y
        up = pts_world[:, 2] - L.cam_h
        level = np.c_[-left, -up, fwd]
        return level @ self.R_src.T

    def render(self, scene: Scene, robot) -> np.ndarray:
        img = self.bg.copy()
        polys = scene_polygons(scene)
        rx, ry, _ = robot
        polys.sort(key=lambda p: -np.hypot(p[0][:, 0].mean() - rx, p[0][:, 1].mean() - ry))
        lim = math.radians(min(self.lens.fov_deg / 2, 100))
        for pts, color in polys:
            pc = self.to_cam(robot, _subdivide(pts, 24))
            if self.lens.kind == "pinhole":
                if (pc[:, 2] < 0.05).any():
                    continue
            elif (np.arctan2(np.hypot(pc[:, 0], pc[:, 1]), pc[:, 2]) > lim).any():
                continue
            uv = self.lens.project(pc)
            if np.abs(uv).max() > 1e5:
                continue
            cv2.fillPoly(img, [np.round(uv).astype(np.int32)], color, lineType=cv2.LINE_AA)
        self.k = (self.k + 1) % len(self.noise)
        img = np.clip(img.astype(np.int16) + self.noise[self.k], 0, 255).astype(np.uint8)
        img[self.outside] = 6                              # вне круга — чёрное
        return cv2.GaussianBlur(img, (3, 3), 0)


def _subdivide(pts: np.ndarray, n: int) -> np.ndarray:
    out = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        for t in np.linspace(0, 1, n, endpoint=False):
            out.append(a + (b - a) * t)
    return np.array(out)


RED, DARK_RED, BLACK = (35, 35, 200), (30, 30, 170), (25, 25, 25)


def _frame_quads(cx, cy, ux, uy, half_w, z0, z1, fw, color):
    """Рамка из четырёх планок в вертикальной плоскости."""
    def q(a0, a1, b0, b1):
        return np.array([(cx + ux * a0, cy + uy * a0, b1), (cx + ux * a1, cy + uy * a1, b1),
                         (cx + ux * a1, cy + uy * a1, b0), (cx + ux * a0, cy + uy * a0, b0)])
    return [(q(-half_w, half_w, z1 - fw, z1), color), (q(-half_w, half_w, z0, z0 + fw), color),
            (q(-half_w, -half_w + fw, z0, z1), color), (q(half_w - fw, half_w, z0, z1), color)]


def _box(cx, cy, ux, uy, nx, ny, half_u, half_n, z0, z1, color):
    """Вертикальный брусок: 4 боковые грани."""
    out = []
    corners = [(-half_u, -half_n), (half_u, -half_n), (half_u, half_n), (-half_u, half_n)]
    for i in range(4):
        (a, b), (c, d) = corners[i], corners[(i + 1) % 4]
        p0 = (cx + ux * a + nx * b, cy + uy * a + ny * b)
        p1 = (cx + ux * c + nx * d, cy + uy * c + ny * d)
        out.append((np.array([(*p0, z1), (*p1, z1), (*p1, z0), (*p0, z0)]), color))
    return out


def scene_polygons(s: Scene):
    nx, ny = math.cos(s.window_yaw), math.sin(s.window_yaw)         # нормаль (к роверу)
    ux, uy = -ny, nx                                                 # вдоль окна
    polys = []
    hw = s.width / 2
    z0, z1 = s.bottom, s.bottom + s.height
    polys += _frame_quads(s.window_x, s.window_y, ux, uy, hw, z0, z1, s.frame_w, RED)
    # Табличка под рамкой и стойки с опорами
    polys.append((np.array([(s.window_x - ux * 0.18, s.window_y - uy * 0.18, z0 - 0.03),
                            (s.window_x + ux * 0.18, s.window_y + uy * 0.18, z0 - 0.03),
                            (s.window_x + ux * 0.18, s.window_y + uy * 0.18, z0 - 0.20),
                            (s.window_x - ux * 0.18, s.window_y - uy * 0.18, z0 - 0.20)]), BLACK))
    for sgn in (-1, 1):
        lx, ly = s.window_x + ux * sgn * s.leg_spacing / 2, s.window_y + uy * sgn * s.leg_spacing / 2
        polys += _box(lx, ly, ux, uy, nx, ny, s.leg_w / 2, s.leg_w / 2, 0.0, z0, BLACK)
        polys += _box(lx, ly, ux, uy, nx, ny, 0.02, 0.15, 0.0, 0.03, BLACK)          # опора буквой Т
    if s.distractors:
        # Сплошной красный ящик сбоку
        bx, by = s.window_x + ux * 1.2 + nx * 0.2, s.window_y + uy * 1.2 + ny * 0.2
        polys += _box(bx, by, ux, uy, nx, ny, 0.15, 0.15, 0.0, 0.3, DARK_RED)
        # Красная рамка того же размера стоит на полу у стены — не окно: низко
        fx, fy = s.window_x - ux * 1.3 - nx * 0.1, s.window_y - uy * 1.3 - ny * 0.1
        polys += _frame_quads(fx, fy, ux, uy, hw, 0.0, s.height, s.frame_w, RED)
        # Мяч
        mx, my = s.window_x - ux * 0.7 + nx * 0.5, s.window_y - uy * 0.7 + ny * 0.5
        ang = np.linspace(0, 2 * np.pi, 20, endpoint=False)
        polys.append((np.c_[mx + ux * 0.1 * np.cos(ang), my + uy * 0.1 * np.cos(ang), 0.1 + 0.1 * np.sin(ang)],
                      (40, 40, 210)))
    return polys


# ---------------------------------------------------------------- попытка


@dataclass
class Result:
    done: bool
    message: str
    time_s: float
    distance_m: float               # от центра ровера до плоскости окна
    axis_offset_m: float            # где ось ровера пересекает плоскость окна, от центра
    yaw_err_deg: float
    path: List[Tuple[float, float, float]] = field(default_factory=list)
    blind_share: float = 0.0        # доля шагов подъезда, когда окна не было в кадре (по одометрии)
    clear_m: float = 0.0            # допуск оси: полуширина проёма

    @property
    def passes(self) -> bool:
        """Признак зачёта: не дальше 1 м, ось проходит через проём окна."""
        return self.done and self.distance_m <= 1.0 and abs(self.axis_offset_m) <= self.clear_m


@dataclass
class Setup:
    """Что знает нода (может отличаться от настоящей сцены и камеры)."""
    lens: LensParams
    geom: WindowGeometry
    params: ApproachParams
    cam_x: float = 0.12
    cam_h: float = 0.10


def default_setup(scene: Scene, lens: Lens, lens_error: bool = False, size_error: float = 0.0) -> Setup:
    if lens.kind == "pinhole":
        lp = LensParams(model="pinhole", hfov_deg=lens.hfov_deg, pitch_deg=lens.pitch_deg)
        if lens_error:                              # угол обзора на 5° больше, наклон на 4° меньше
            lp.hfov_deg += 5
            lp.pitch_deg -= 4
    else:
        lp = LensParams(model="ideal", fov_deg=lens.fov_deg, circle_r=lens.circle_r, pitch_deg=lens.pitch_deg)
        if lens_error:                              # угол объектива на 10° меньше, наклон не учтён
            lp.fov_deg -= 10
            lp.pitch_deg = 0.0
    g = WindowGeometry(width_m=scene.width * (1 + size_error), height_m=scene.height * (1 + size_error),
                       frame_m=scene.frame_w, bottom_m=scene.bottom)
    # Веб-камера видит окно 85 × 70 см (центр на 1,2 м) примерно до 0,8 м — там и останавливаемся
    params = ApproachParams(stop_distance_m=0.8 if lens.kind == "pinhole" else 0.6)
    return Setup(lp, g, params, lens.cam_x, lens.cam_h)


def run(start, scene: Optional[Scene] = None, lens: Optional[Lens] = None, setup: Optional[Setup] = None,
        seed: int = 0, fps: float = 10.0, camera: Optional[Camera] = None) -> Result:
    scene = scene or Scene()
    lens = lens or WEBCAM
    setup = setup or default_setup(scene, lens)
    rng = random.Random(seed)
    camera = camera or Camera(lens, seed)
    rect = Rectifier(setup.lens)
    cam = CameraModel.rectified(rect, (lens.width, lens.height), setup.cam_x, 0.0, setup.cam_h)
    ap = WindowApproach(setup.params)
    ap.start()
    x, y, yaw = start
    dt = 1.0 / fps
    path = [(x, y, yaw)]
    pending = None                                                       # кадр приходит с задержкой
    odom = None
    t = 0.0
    blind_steps = approach_steps = 0
    while ap.state not in ("done", "failed") and t < setup.params.timeout_s + 1:
        img = rect.rectify(camera.render(scene, (x, y, yaw)))
        prior = ap.pose if ap.state == "approach" else None
        _, pose = find_window(img, cam, setup.geom, prior=prior)
        meas, pending = pending, pose
        cmd = ap.update(meas, dt, odom)
        if ap.state == "approach":
            approach_steps += 1
            blind_steps += ap.source == "одометрия"
        # Меканум: команда в осях ровера, исполняется неточно (±10%) и с проскальзыванием
        k = 1 + rng.gauss(0, 0.05)
        vx, vy, wz = cmd.vx * k, cmd.vy * k * 0.9, cmd.wz * (1 + rng.gauss(0, 0.05))
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        yaw += wz * dt
        # Одометрия колёс: настоящее движение с ошибкой ~3% (её дельта придёт на следующем шаге)
        odom = (vx * dt * (1 + rng.gauss(0, 0.03)), vy * dt * (1 + rng.gauss(0, 0.03)), wz * dt * (1 + rng.gauss(0, 0.03)))
        t += dt
        path.append((x, y, yaw))
    # Истинное положение относительно окна
    nx, ny = math.cos(scene.window_yaw), math.sin(scene.window_yaw)
    dist = (x - scene.window_x) * nx + (y - scene.window_y) * ny
    hx, hy = math.cos(yaw), math.sin(yaw)
    denom = hx * nx + hy * ny
    if abs(denom) < 1e-6:
        off = math.inf
    else:
        tt = -dist / denom
        px, py = x + hx * tt, y + hy * tt
        off = (px - scene.window_x) * (-ny) + (py - scene.window_y) * nx
    face = math.atan2(-ny, -nx)
    yaw_err = math.degrees((yaw - face + math.pi) % (2 * math.pi) - math.pi)
    return Result(ap.state == DONE, ap.message, t, abs(dist), off, yaw_err, path,
                  blind_steps / max(approach_steps, 1), scene.width / 2 - scene.frame_w)


STARTS = [  # (x, y, yaw) ровера; окно в (0,0) смотрит на +x
    (2.5, 0.0, math.pi), (2.5, 1.0, math.pi), (2.5, -1.0, math.pi), (3.0, 0.5, math.pi + 0.5),
    (2.0, -0.8, math.pi - 0.6), (1.6, 1.2, math.pi - 0.3), (3.0, -1.5, math.pi + 0.7),
    (1.5, 0.0, math.pi + 0.9),                      # окно сбоку, на краю кадра или вне его
    (2.5, 0.0, 0.0),                                # стоит спиной — нужен поиск
]


def chessboard_frames(lens: Lens, n: int = 14, cols: int = 9, rows: int = 6, square: float = 0.04, seed: int = 0):
    """Снимки шахматной доски (cols×rows внутренних углов) этим объективом — для проверки lens_calib."""
    cam = Camera(lens, seed)
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        d = rng.uniform(0.3, 0.5)
        rvec = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.6, 0.6), rng.uniform(-0.3, 0.3)])
        R = cv2.Rodrigues(rvec)[0]
        tvec = np.array([*(rng.uniform(-0.4, 0.4, 2) * d), d])
        img = np.full((lens.height, lens.width, 3), 30, np.uint8)
        # Белое поле доски, на нём чёрные клетки
        border = (np.array([[-1, -1, 0], [cols + 1, -1, 0], [cols + 1, rows + 1, 0], [-1, rows + 1, 0]], float)
                  - [(cols - 1) / 2 + 0.5, (rows - 1) / 2 + 0.5, 0]) * square
        pts = _subdivide(border, 30) @ R.T + tvec
        cv2.fillPoly(img, [np.round(lens.project(pts)).astype(np.int32)], (235, 235, 235), cv2.LINE_AA)
        for i in range(cols + 1):
            for j in range(rows + 1):
                if (i + j) % 2:
                    continue
                sq = (np.array([[i, j, 0], [i + 1, j, 0], [i + 1, j + 1, 0], [i, j + 1, 0]], float)
                      - [(cols - 1) / 2 + 0.5, (rows - 1) / 2 + 0.5, 0]) * square
                pts = _subdivide(sq, 8) @ R.T + tvec
                cv2.fillPoly(img, [np.round(lens.project(pts)).astype(np.int32)], (20, 20, 20), cv2.LINE_AA)
        img[cam.outside] = 6
        out.append(cv2.GaussianBlur(img, (3, 3), 0))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", choices=("webcam", "fisheye"), default="webcam")
    ap.add_argument("--cam-height", type=float, help="высота камеры над полом, м (по умолчанию 0,45 / 0,10)")
    ap.add_argument("--pitch", type=float, help="наклон камеры вверх, ° (по умолчанию 20 / 4)")
    ap.add_argument("--stop", type=float, help="остановка: от центра ровера до плоскости окна, м")
    ap.add_argument("--show", help="сохранить картинку траекторий (PNG)")
    ap.add_argument("--frame", help="сохранить кадр камеры и выпрямленный кадр (PNG)")
    ap.add_argument("--lens-error", action="store_true", help="угол обзора и наклон камеры заданы с ошибкой")
    ap.add_argument("--size-error", type=float, default=0.0, help="ошибка размеров окна, доля (0.1 = +10%%)")
    ap.add_argument("--no-clutter", action="store_true", help="без красных помех")
    a = ap.parse_args()
    lens = WEBCAM if a.camera == "webcam" else FISHEYE
    if a.cam_height is not None:
        lens = replace(lens, cam_h=a.cam_height)
    if a.pitch is not None:
        lens = replace(lens, pitch_deg=a.pitch)
    camera = Camera(lens, 1)
    results = []
    for i, st in enumerate(STARTS):
        for wyaw in (0.0, 0.35, -0.35):           # окно стоит прямо или повёрнуто на ±20°
            scene = Scene(window_yaw=wyaw, distractors=not a.no_clutter)
            setup = default_setup(scene, lens, a.lens_error, a.size_error)
            if a.stop:
                setup.params.stop_distance_m = a.stop
            r = run(st, scene, lens, setup, seed=i * 7 + int(wyaw * 10), camera=camera)
            results.append((st, wyaw, r))
            print(f"старт ({st[0]:+.1f}, {st[1]:+.1f}, {math.degrees(st[2]) % 360:5.0f}°), окно {math.degrees(wyaw):+4.0f}°: "
                  f"{'ЗАЧЁТ' if r.passes else 'нет  '} {r.time_s:5.1f} с, до окна {r.distance_m:.2f} м, "
                  f"ось {r.axis_offset_m * 100:+5.1f} см, угол {r.yaw_err_deg:+5.1f}°, "
                  f"вслепую {r.blind_share * 100:3.0f}% — {r.message}", flush=True)
    ok = sum(r.passes for _, _, r in results)
    print(f"ИТОГ: зачёт в {ok} из {len(results)} попыток")
    if a.show:
        _plot(results, a.show)
    if a.frame:
        save_frame(a.frame, camera)
    return 0 if ok == len(results) else 1


def save_frame(path, camera: Camera, robot=(2.0, 0.9, math.pi - 0.25), window_yaw=0.35):
    """Кадр камеры и выпрямленный кадр: зелёное — окно, оранжевое — отброшенные рамки."""
    lens = camera.lens
    scene = Scene(window_yaw=window_yaw)
    src = camera.render(scene, robot)
    setup = default_setup(scene, lens)
    rect = Rectifier(setup.lens)
    img = rect.rectify(src)
    cam = CameraModel.rectified(rect, (lens.width, lens.height), setup.cam_x, 0.0, setup.cam_h)
    det, pose = find_window(img, cam, setup.geom)
    dbg = draw_debug(img, det, WindowApproach())
    for c in detect_candidates(img):
        if det is None or np.abs(c.corners - det.corners).max() > 1:
            cv2.polylines(dbg, [c.corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 165, 255), 2)
    left = cv2.resize(src, (int(lens.width * dbg.shape[0] / lens.height), dbg.shape[0]))
    cv2.imwrite(path, np.hstack([left, dbg]))
    return det, pose


def _plot(results, path):
    S, W = 120, 900
    img = np.full((W, W, 3), 255, np.uint8)

    def P(x, y):
        return int(W / 2 + (x - 1.0) * S), int(W / 2 - y * S)

    colors = [(31, 119, 180), (255, 127, 14), (44, 160, 44), (214, 39, 40), (148, 103, 189),
              (140, 86, 75), (227, 119, 194), (127, 127, 127), (188, 189, 34)]
    for k, (st, wyaw, r) in enumerate(results):
        if wyaw != 0.0:
            continue
        c = colors[(k // 3) % len(colors)][::-1]
        pts = np.array([P(x, y) for x, y, _ in r.path], np.int32)
        cv2.polylines(img, [pts], False, c, 2)
        cv2.circle(img, P(*r.path[0][:2]), 5, c, -1)
    cv2.line(img, P(0, -0.25), P(0, 0.25), (0, 0, 220), 6)
    cv2.line(img, P(1.0, -2), P(1.0, 2), (200, 200, 200), 1)
    cv2.putText(img, "okno", (P(0, 0)[0] - 70, P(0, 0)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 200), 2)
    cv2.putText(img, "1 m", (P(1.0, 2)[0] + 4, P(1.0, 2)[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
    cv2.imwrite(path, img)


if __name__ == "__main__":
    raise SystemExit(main())
