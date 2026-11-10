"""Подъезд ровера к красному окну — логика без ROS.

Цель кейса 3: ровер сам находит камерой красное окно (рамку на стойках), подъезжает и
останавливается не дальше 1 м от него так, чтобы продольная ось ровера проходила через окно.
Ровер на мекануме едет и вбок, поэтому выравнивается сразу по трём осям: вперёд, вбок, поворот.

Как это работает
0. Кадр выпрямляется в ровную камеру-обскуру (``lens.Rectifier``): у «рыбьего глаза» ровера —
   бочка и наклон, у веб-камеры на второй Raspberry Pi — только наклон.
1. ``detect_candidates`` — красная маска в HSV, контуры с дыркой (рамка), четырёхугольники.
2. ``estimate_pose`` — положение окна относительно ровера по одному кадру. Высота каждого
   вертикального края в пикселях даёт расстояние до этого края (известна высота рамки),
   горизонтальная координата края — его направление. Два края — две точки на полу; середина —
   центр окна, перпендикуляр — его нормаль.
   ``find_window`` оставляет только рамки, у которых ширина и высота над полом сходятся с окном
   (``WindowGeometry``), и из них — ближайшую к прошлой оценке: красные предметы на площадке
   не мешают.
3. ``WindowApproach.update`` — цель: точка на нормали в ``stop_distance`` перед окном, ровер
   смотрит в окно. Ошибки вперёд, вбок и по углу гасятся пропорционально, с ограничением
   скоростей. Пока ровер далеко от оси окна, он смотрит на центр окна (окно не уходит из кадра,
   а едет он и вбок); у оси — разворачивается по нормали. Окно пропало из кадра ненадолго — едем
   по последней оценке, сдвинутой на собственное движение (одометрия колёс или последняя
   команда); надолго — поиск поворотом.
4. Вблизи окно может уйти за верх кадра (оно высоко). Если ровер к этому моменту уже выровнен и
   до окна меньше ``blind_finish_m``, остаток пути он проходит по одометрии; не выровнен —
   отъезжает назад, пока окно снова не влезет в кадр. Веб-камера выше и с наклоном вверх видит
   окно до самой остановки.

Оси: ровер — x вперёд, y влево, угол против часовой (как base_link в ROS). Камера смотрит вперёд.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------- распознавание


@dataclass
class Detection:
    corners: np.ndarray            # 4×2 пикселя: левый верх, правый верх, правый низ, левый низ
    area: float                    # площадь внешнего контура, px²
    squareness: float              # 1 — квадрат в кадре; меньше — сильный ракурс
    touches_border: bool           # край рамки упёрся в границу кадра — оценка ненадёжна
    edge_h: Optional[Tuple[float, float]] = None   # высота левого и правого края, px (с долями)

    @property
    def center(self) -> Tuple[float, float]:
        c = self.corners.mean(axis=0)
        return float(c[0]), float(c[1])


@dataclass
class DetectorParams:
    # Красный в HSV OpenCV (H 0..180) — два участка вокруг нуля
    h_low1: int = 0
    h_high1: int = 10
    h_low2: int = 165
    h_high2: int = 180
    s_min: int = 90
    v_min: int = 60
    min_area_frac: float = 0.001   # окно не меньше этой доли кадра
    min_hole_frac: float = 0.15    # дырка рамки — не меньше этой доли внешнего контура
    max_aspect: float = 3.0        # ширина/высота в кадре (ракурс сжимает квадрат)
    require_hole: bool = True      # False — годится и сплошной красный квадрат
    morph_px: int = 3              # сглаживание маски; больше — съедает тонкие планки рамки издалека


def _order_corners(pts: np.ndarray) -> np.ndarray:
    """Упорядочить 4 точки: ЛВ, ПВ, ПН, ЛН (y вниз)."""
    pts = pts.reshape(4, 2).astype(np.float64)
    s = pts.sum(axis=1)
    d = pts[:, 0] - pts[:, 1]
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmax(d)]
    bl = pts[np.argmin(d)]
    out = np.array([tl, tr, br, bl])
    if len({tuple(p) for p in out.tolist()}) < 4:   # вырожденный случай — по углу от центра
        c = pts.mean(axis=0)
        ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
        out = pts[np.argsort(ang)]                   # -π..π: ЛВ, ПВ, ПН, ЛН при y вниз
        out = np.roll(out, -int(np.argmin(out.sum(axis=1))), axis=0)
    return out


def _refine_corners(contour: np.ndarray, corners: np.ndarray, band_px: float = 3.0) -> np.ndarray:
    """Углы с точностью до долей пикселя: прямая по всем точкам контура вдоль каждой стороны,
    углы — пересечения соседних прямых. Расстояние считается по высоте края в пикселях, и ошибка
    в 1 px на краю высотой 50 px — это уже 2% расстояния и ~7° разворота окна."""
    pts = contour.reshape(-1, 2).astype(np.float64)
    lines = []
    for i in range(4):
        a, b = corners[i], corners[(i + 1) % 4]
        d = b - a
        L = float(np.hypot(*d))
        if L < 8:
            return corners
        u = d / L
        rel = pts - a
        t = rel @ u
        dist = np.abs(rel[:, 0] * u[1] - rel[:, 1] * u[0])
        sel = pts[(dist < band_px) & (t > 0.15 * L) & (t < 0.85 * L)]
        if len(sel) < 5:
            return corners
        vx, vy, x0, y0 = cv2.fitLine(sel.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
        lines.append((np.array([x0, y0], np.float64), np.array([vx, vy], np.float64)))
    out = np.empty((4, 2))
    for i in range(4):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]          # стороны, сходящиеся в углу i
        A = np.array([d1, -d2]).T
        if abs(np.linalg.det(A)) < 1e-6:
            return corners
        t = np.linalg.solve(A, p2 - p1)
        out[i] = p1 + d1 * t[0]
    if np.abs(out - corners).max() > 4 * band_px:           # что-то пошло не так — оставить как было
        return corners
    return out


def _redness(bgr: np.ndarray) -> np.ndarray:
    """Насколько пиксель красный: R − max(G, B). Плавно растёт на краю рамки — по сумме вдоль
    столбца высота края считается с долями пикселя."""
    b, g, r = cv2.split(bgr.astype(np.int16))
    return (r - np.maximum(g, b)).astype(np.float32)


def _edge_heights(redness: np.ndarray, corners: np.ndarray, hole: np.ndarray) -> Optional[Tuple[float, float]]:
    """Высота левого и правого края рамки, px с долями.

    По каждому столбцу над дыркой ищется внешний край верхней и нижней планки с долями пикселя
    (по тому, «сколько красного» в пограничных пикселях), через эти точки — прямые верха и низа.
    Высота края = расстояние между прямыми на его x. Прямые — изображения горизонтальных краёв
    рамки, их наклон зависит от разворота окна, поэтому и разворот, и расстояние берутся по всей
    длине верха и низа, а не по четырём углам (углы — целые пиксели контура: на краю высотой 50 px
    ошибка в 1 px — это 2% расстояния и ~7° разворота)."""
    H, W = redness.shape
    hx, hy, hw, hh = cv2.boundingRect(hole)

    def y_at(pa, pb, x):
        if abs(pb[0] - pa[0]) < 1e-6:
            return (pa[1] + pb[1]) / 2
        return pa[1] + (pb[1] - pa[1]) * (x - pa[0]) / (pb[0] - pa[0])

    tops, bots = [], []
    for x in range(hx + 2, hx + hw - 2):
        if not 0 <= x < W:
            continue
        col = redness[:, x]
        for pa, pb, out, sign in ((corners[0], corners[1], tops, -1), (corners[3], corners[2], bots, 1)):
            e0 = y_at(pa, pb, x)
            ya, yb = int(round(e0)) - 4, int(round(e0)) + 5
            if ya - 3 < 0 or yb + 3 > H:
                continue
            seg = col[ya:yb]
            if sign < 0:                     # верх: снаружи (выше) фон, внутри (ниже) планка
                bg = float(np.median(col[ya - 3:ya]))
                fg = float(np.median(col[yb - 2:yb + 1]))
            else:                            # низ: внутри (выше) планка, снаружи (ниже) фон
                fg = float(np.median(col[ya - 1:ya + 2]))
                bg = float(np.median(col[yb:yb + 3]))
            if fg - bg < 20:
                continue
            frac = float(np.clip((seg - bg) / (fg - bg), 0, 1).sum())
            out.append((x, yb - frac if sign < 0 else ya + frac))
    if len(tops) < 4 or len(bots) < 4:
        return None
    t, b = np.array(tops), np.array(bots)
    mt, ct = np.polyfit(t[:, 0], t[:, 1], 1)
    mb, cb = np.polyfit(b[:, 0], b[:, 1], 1)
    res = []
    for top, bottom in ((corners[0], corners[3]), (corners[1], corners[2])):
        x = (top[0] + bottom[0]) / 2
        res.append(float((mb * x + cb) - (mt * x + ct)))
    if min(res) < 4:
        return None
    return res[0], res[1]


def red_mask(bgr: np.ndarray, p: DetectorParams) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (p.h_low1, p.s_min, p.v_min), (p.h_high1, 255, 255))
    m2 = cv2.inRange(hsv, (p.h_low2, p.s_min, p.v_min), (p.h_high2, 255, 255))
    m = cv2.bitwise_or(m1, m2)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (p.morph_px, p.morph_px))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return m


def detect_candidates(bgr: np.ndarray, p: Optional[DetectorParams] = None) -> List[Detection]:
    """Все красные рамки (контур с дыркой, четырёхугольник), крупные первыми."""
    p = p or DetectorParams()
    h, w = bgr.shape[:2]
    mask = red_mask(bgr, p)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    out: List[Detection] = []
    redness = None
    for i, c in enumerate(contours):
        if hierarchy[i][3] != -1:                       # внутренний контур — пропускаем
            continue
        area = cv2.contourArea(c)
        if area < p.min_area_frac * w * h:
            continue
        # Дырка: самый большой дочерний контур
        hole = 0.0
        hole_c = None
        child = hierarchy[i][2]
        while child != -1:
            a_ch = cv2.contourArea(contours[child])
            if a_ch > hole:
                hole, hole_c = a_ch, contours[child]
            child = hierarchy[child][0]
        if p.require_hole and hole < p.min_hole_frac * area:
            continue
        hull = cv2.convexHull(c)
        peri = cv2.arcLength(hull, True)
        approx = None
        for eps in (0.02, 0.03, 0.045, 0.06):
            a = cv2.approxPolyDP(hull, eps * peri, True)
            if len(a) == 4:
                approx = a
                break
        if approx is None:
            continue
        corners = _refine_corners(c, _order_corners(approx))
        top = np.linalg.norm(corners[1] - corners[0])
        bottom = np.linalg.norm(corners[2] - corners[3])
        left = np.linalg.norm(corners[3] - corners[0])
        right = np.linalg.norm(corners[2] - corners[1])
        width, height = (top + bottom) / 2, (left + right) / 2
        if height < 1 or width < 1:
            continue
        aspect = width / height
        if aspect > p.max_aspect or aspect < 1.0 / p.max_aspect:
            continue
        # Вертикальные края примерно вертикальны (окно стоит), прямоугольность контура
        fill = area / max(cv2.contourArea(approx), 1.0)
        if fill < 0.8:
            continue
        squareness = min(aspect, 1 / aspect)
        margin = 3
        touches = bool((corners[:, 0] <= margin).any() or (corners[:, 0] >= w - 1 - margin).any()
                       or (corners[:, 1] <= margin).any() or (corners[:, 1] >= h - 1 - margin).any())
        edge_h = None
        if hole_c is not None:
            if redness is None:
                redness = _redness(bgr)
            edge_h = _edge_heights(redness, corners, hole_c)
        out.append(Detection(corners=corners, area=float(area), squareness=float(squareness), touches_border=touches,
                             edge_h=edge_h))
    out.sort(key=lambda d: -d.area)
    return out


def detect_window(bgr: np.ndarray, p: Optional[DetectorParams] = None) -> Optional[Detection]:
    """Самая крупная красная рамка в кадре или None (без проверки размеров — см. find_window)."""
    c = detect_candidates(bgr, p)
    return c[0] if c else None


# ---------------------------------------------------------------- геометрия


@dataclass
class CameraModel:
    """Камера-обскура, смотрит горизонтально вперёд. Для «рыбьего глаза» — выпрямленный кадр
    (lens.Rectifier): тогда f_px и главная точка заданы явно."""
    width: int
    height: int
    hfov_deg: float                # горизонтальный угол обзора (если f_px не задан)
    x_offset_m: float = 0.0        # камера впереди центра ровера, м
    y_offset_m: float = 0.0        # камера левее центра ровера, м
    z_m: float = 0.10              # высота объектива над полом, м
    f_px: float = 0.0
    cx_px: float = -1.0
    cy_px: float = -1.0

    @property
    def f(self) -> float:
        if self.f_px > 0:
            return self.f_px
        return (self.width / 2) / math.tan(math.radians(self.hfov_deg) / 2)

    @property
    def cx(self) -> float:
        return self.cx_px if self.cx_px >= 0 else (self.width - 1) / 2

    @property
    def cy(self) -> float:
        return self.cy_px if self.cy_px >= 0 else (self.height - 1) / 2

    @classmethod
    def rectified(cls, rect, src_size, x_offset_m=0.0, y_offset_m=0.0, z_m=0.10) -> "CameraModel":
        w, h, f, cx, cy = rect.out_camera(src_size)
        return cls(w, h, math.degrees(2 * math.atan((w / 2) / f)), x_offset_m, y_offset_m, z_m, f, cx, cy)


@dataclass
class WindowGeometry:
    """Окно на площадке. Расстояние считается по высоте рамки; ширина и высота над полом — проверка,
    что найдено именно окно, а не другой красный предмет."""
    width_m: float = 0.50          # внешний размер красной рамки по горизонтали
    height_m: float = 0.40         #                                по вертикали
    frame_m: float = 0.05          # ширина красной планки (для зачёта: ось — внутри проёма)
    bottom_m: float = 0.0          # низ рамки над полом; 0 — не задан (проверка только bottom_min_m)
    bottom_min_m: float = 0.30     # окно не ниже этого (отсекает красное на полу)
    bottom_tol_m: float = 0.15     # допуск проверки высоты, если bottom_m задан ...
    bottom_tol_deg: float = 4.0    # ... плюс столько градусов ошибки наклона камеры (растёт с расстоянием)
    width_tol: float = 0.35        # допуск ширины, доля


@dataclass
class WindowPose:
    """Окно относительно ровера (м, рад): центр, нормаль к ровeру, расстояние."""
    cx: float
    cy: float
    nx: float                      # единичная нормаль окна, смотрит на ровер
    ny: float
    left: Tuple[float, float]
    right: Tuple[float, float]

    @property
    def distance(self) -> float:
        """Расстояние от ровера до плоскости окна, м."""
        return abs(self.cx * self.nx + self.cy * self.ny)

    @property
    def yaw_to_face(self) -> float:
        """На сколько повернуться, чтобы смотреть в окно (рад, + против часовой)."""
        return math.atan2(-self.ny, -self.nx)

    def axis_offset(self) -> float:
        """Где ось ровера (луч вперёд) пересекает плоскость окна: смещение от центра окна, м.
        inf — ось параллельна окну."""
        # Луч (t, 0); плоскость n·(p - c) = 0 → t = n·c / n_x
        if abs(self.nx) < 1e-6:
            return math.inf
        t = (self.nx * self.cx + self.ny * self.cy) / self.nx
        # точка пересечения (t, 0); смещение вдоль окна
        ux, uy = -self.ny, self.nx
        return (t - self.cx) * ux + (0 - self.cy) * uy

    def moved(self, dx: float, dy: float, dyaw: float) -> "WindowPose":
        """Та же поза окна после собственного движения ровера на (dx, dy, dyaw) в старых осях."""
        c, s = math.cos(-dyaw), math.sin(-dyaw)

        def tr(px, py):
            px, py = px - dx, py - dy
            return c * px - s * py, s * px + c * py

        cx, cy = tr(self.cx, self.cy)
        nx, ny = c * self.nx - s * self.ny, s * self.nx + c * self.ny
        return WindowPose(cx, cy, nx, ny, tr(*self.left), tr(*self.right))


def estimate_pose(det: Detection, cam: CameraModel, height_m: float) -> WindowPose:
    """Положение окна (внешняя высота рамки height_m) относительно ровера по одному кадру."""
    tl, tr, br, bl = det.corners
    f = cam.f

    def edge_point(top, bottom, h_sub=None):
        h_px = max(h_sub if h_sub else abs(bottom[1] - top[1]), 1.0)
        depth = height_m * f / h_px                          # вдоль оптической оси
        u = (top[0] + bottom[0]) / 2 - cam.cx                # вправо в кадре
        lateral = u * depth / f                              # вправо, м
        return cam.x_offset_m + depth, cam.y_offset_m - lateral

    L = edge_point(tl, bl, det.edge_h[0] if det.edge_h else None)
    R = edge_point(tr, br, det.edge_h[1] if det.edge_h else None)
    cx, cy = (L[0] + R[0]) / 2, (L[1] + R[1]) / 2
    ux, uy = R[0] - L[0], R[1] - L[1]
    n = math.hypot(ux, uy) or 1.0
    nx, ny = uy / n, -ux / n                                 # перпендикуляр к линии окна
    if nx * (0 - cx) + ny * (0 - cy) < 0:                    # нормаль — в сторону ровера
        nx, ny = -nx, -ny
    return WindowPose(cx, cy, nx, ny, L, R)


def bottom_height(det: Detection, cam: CameraModel, height_m: float) -> float:
    """Высота низа рамки над полом по кадру, м (среднее по двум краям)."""
    tl, tr, br, bl = det.corners
    zs = []
    for top, bottom in ((tl, bl), (tr, br)):
        depth = height_m * cam.f / max(abs(bottom[1] - top[1]), 1.0)
        zs.append(cam.z_m + depth * (cam.cy - bottom[1]) / cam.f)
    return float(sum(zs) / 2)


def check_window(det: Detection, pose: WindowPose, cam: CameraModel, g: WindowGeometry) -> Optional[str]:
    """None — похоже на окно; иначе почему нет."""
    width = math.hypot(pose.right[0] - pose.left[0], pose.right[1] - pose.left[1])
    if abs(width - g.width_m) > g.width_tol * g.width_m:
        return f"ширина {width:.2f} м вместо {g.width_m:.2f}"
    z = bottom_height(det, cam, g.height_m)
    tol = g.bottom_tol_m + pose.distance * math.tan(math.radians(g.bottom_tol_deg))
    if g.bottom_m > 0 and abs(z - g.bottom_m) > tol:
        return f"низ на {z:.2f} м вместо {g.bottom_m:.2f}"
    if z < g.bottom_min_m:
        return f"низ на {z:.2f} м — слишком низко"
    return None


def find_window(bgr: np.ndarray, cam: CameraModel, g: WindowGeometry, dp: Optional[DetectorParams] = None,
                prior: Optional[WindowPose] = None):
    """Окно в (выпрямленном) кадре: (Detection, WindowPose) или (Detection|None, None).
    Из красных рамок берутся только те, что сходятся с размерами и высотой окна; из них — ближайшая
    к прошлой оценке (если есть), иначе самая крупная. Обрезанная краем кадра рамка возвращается
    без позы (расстояние по ней неверно)."""
    cands = detect_candidates(bgr, dp)
    best = None
    cut = None
    for det in cands:
        if det.touches_border:
            cut = cut or det
            continue
        pose = estimate_pose(det, cam, g.height_m)
        if check_window(det, pose, cam, g) is not None:
            continue
        if prior is not None:
            key = math.hypot(pose.cx - prior.cx, pose.cy - prior.cy)
        else:
            key = -det.area
        if best is None or key < best[0]:
            best = (key, det, pose)
    if best is None:
        # Окна нет, но рамку показать стоит: в отладочном кадре и для ~/calibrate (при сильно неверном
        # наклоне или угле обзора проверка размеров не проходит, а калибровка как раз это и исправит)
        whole = [d for d in cands if not d.touches_border]
        return (whole[0] if whole else cut), None
    return best[1], best[2]


# ---------------------------------------------------------------- регулятор


@dataclass
class ApproachParams:
    stop_distance_m: float = 0.6   # где остановиться перед окном (кейс: не дальше 1 м)
    tol_xy_m: float = 0.05         # допуск положения
    tol_yaw_deg: float = 3.0       # допуск угла
    settle_s: float = 0.5          # столько держаться в допуске, чтобы закончить
    k_x: float = 0.8
    k_y: float = 1.0
    k_yaw: float = 1.5
    max_vx: float = 0.20           # м/с
    max_vy: float = 0.20
    max_wz: float = 0.6            # рад/с
    min_v: float = 0.03            # меньше — моторы не трогаются; держим хотя бы столько
    lost_hold_s: float = 1.0       # окно пропало: столько едем по последней оценке
    blind_finish_m: float = 1.3    # ближе этого и выровнен — доводка по одометрии без кадров
    blind_axis_m: float = 0.10     # «выровнен»: ось ровера не дальше этого от центра окна
    blind_yaw_deg: float = 10.0    #             и угол к нормали не больше этого
    blind_max_s: float = 10.0      # доводка вслепую не дольше
    backoff_v: float = 0.08        # окно потеряно вблизи, а ровер не выровнен: отъехать назад, м/с
    backoff_s: float = 3.0         #   не дольше, потом поиск поворотом
    look_blend_m: float = 0.4      # дальше этого от оси — смотреть на центр окна, ближе — по нормали
    search_wz: float = 0.35        # поиск: поворот на месте, рад/с
    search_timeout_s: float = 25.0 # полный оборот и ещё немного
    timeout_s: float = 90.0        # вся попытка
    smoothing: float = 0.5         # сглаживание оценки (0 — без, 1 — не обновлять)


@dataclass
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


IDLE, SEARCH, APPROACH, DONE, FAILED = "idle", "search", "approach", "done", "failed"


@dataclass
class WindowApproach:
    p: ApproachParams = field(default_factory=ApproachParams)
    state: str = IDLE
    pose: Optional[WindowPose] = None
    message: str = ""
    errors: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    _t: float = 0.0
    _seen_t: float = -1e9
    _search_t: float = 0.0
    _in_tol_t: float = 0.0
    _blind: bool = False
    _blind_t: float = 0.0
    _backoff_t: float = 0.0
    _last_cmd: Command = field(default_factory=Command)
    source: str = ""               # откуда последняя оценка: камера | одометрия

    def start(self) -> None:
        self.state, self.pose, self.message = SEARCH, None, "поиск окна"
        self._t = self._search_t = self._in_tol_t = self._blind_t = self._backoff_t = 0.0
        self._seen_t = -1e9
        self._blind = False
        self._last_cmd = Command()

    def stop(self, msg: str = "остановлено") -> None:
        self.state, self.message, self._last_cmd = IDLE, msg, Command()

    def update(self, measurement: Optional[WindowPose], dt: float,
               odom_delta: Optional[Tuple[float, float, float]] = None) -> Command:
        """Шаг регулятора: новая оценка окна по камере (или None), прошедшее время и, если есть,
        собственное движение ровера за это время по одометрии (dx, dy, dyaw в осях ровера на начало
        шага). Возвращает команду скорости в осях ровера."""
        if self.state in (IDLE, DONE, FAILED):
            return Command()
        self._t += dt
        # Собственное движение за шаг: одометрия, а без неё — последняя команда
        if self.pose is not None:
            if odom_delta is not None:
                self.pose = self.pose.moved(*odom_delta)
            else:
                c = self._last_cmd
                self.pose = self.pose.moved(c.vx * dt, c.vy * dt, c.wz * dt)
        if measurement is not None:
            a, self.source = self.p.smoothing, "камера"
            self._seen_t = self._t
            self._blind = False
            self._backoff_t = 0.0
            if self.pose is None or self.state == SEARCH:
                self.pose = measurement
            else:
                self.pose = WindowPose(
                    a * self.pose.cx + (1 - a) * measurement.cx, a * self.pose.cy + (1 - a) * measurement.cy,
                    *_unit(a * self.pose.nx + (1 - a) * measurement.nx, a * self.pose.ny + (1 - a) * measurement.ny),
                    measurement.left, measurement.right)
            if self.state == SEARCH:
                self.state, self.message = APPROACH, "окно найдено, подъезжаю"
        if self._t > self.p.timeout_s:
            return self._finish(FAILED, "не успел за отведённое время")

        lost = self.state == APPROACH and self._t - self._seen_t > self.p.lost_hold_s
        if lost and self.pose is not None:
            self.source = "одометрия"
        if lost and not self._blind and self.pose is not None and self._aligned_close():
            self._blind, self._blind_t, self.message = True, 0.0, "окно ушло из кадра вблизи — довожу по одометрии"
        if self._blind and lost:
            self._blind_t += dt
            if self._blind_t > self.p.blind_max_s:
                return self._finish(FAILED, "доводка по одометрии не сошлась")
            lost = False
        # Потеряно вблизи, но не выровнен: окно просто не влезает в кадр — отъехать назад
        if lost and self.pose is not None and self.pose.distance < self.p.blind_finish_m \
                and self._backoff_t < self.p.backoff_s:
            self._backoff_t += dt
            self.message = "окно не влезает в кадр — отъезжаю назад"
            return self._out(Command(-self.p.backoff_v, 0.0, 0.0))
        if self.state == SEARCH or lost:
            if self.state == APPROACH:
                self.state, self.message, self._search_t = SEARCH, "окно потеряно, ищу", 0.0
            self._search_t += dt
            if self._search_t > self.p.search_timeout_s:
                return self._finish(FAILED, "окно не найдено")
            # Ищем в ту сторону, где окно видели последний раз
            wz = self.p.search_wz
            if self.pose is not None and math.atan2(self.pose.cy, self.pose.cx) < 0:
                wz = -wz
            return self._out(Command(0.0, 0.0, wz))

        w = self.pose
        gx = w.cx + w.nx * self.p.stop_distance_m              # точка остановки
        gy = w.cy + w.ny * self.p.stop_distance_m
        # Куда смотреть: далеко от оси — на центр окна, у оси — по нормали
        off = abs(w.axis_offset()) if math.isfinite(w.axis_offset()) else 1e9
        lateral = abs((0 - w.cx) * (-w.ny) + (0 - w.cy) * w.nx)       # от ровера до оси окна
        beta = max(0.0, min(1.0, 1.0 - lateral / self.p.look_blend_m))
        bearing = math.atan2(w.cy, w.cx)
        gyaw = beta * w.yaw_to_face + (1 - beta) * bearing
        self.errors = (gx, gy, w.yaw_to_face)
        del off
        in_tol = (math.hypot(gx, gy) < self.p.tol_xy_m and abs(w.yaw_to_face) < math.radians(self.p.tol_yaw_deg))
        if in_tol:
            self._in_tol_t += dt
            if self._in_tol_t >= self.p.settle_s:
                return self._finish(DONE, f"стою перед окном: {w.distance:.2f} м, ось смещена на "
                                          f"{w.axis_offset() * 100:.0f} см")
            return self._out(Command())
        self._in_tol_t = 0.0
        # Пока сильно смотрим мимо — меньше едем, больше поворачиваем (окно не уходит из кадра)
        turn_scale = max(0.3, math.cos(min(abs(gyaw), math.pi / 2)))
        cmd = Command(
            _limit(self.p.k_x * gx * turn_scale, self.p.max_vx, self.p.min_v, self.p.tol_xy_m / 2, gx),
            _limit(self.p.k_y * gy * turn_scale, self.p.max_vy, self.p.min_v, self.p.tol_xy_m / 2, gy),
            _clip(self.p.k_yaw * gyaw, self.p.max_wz))
        return self._out(cmd)

    def _aligned_close(self) -> bool:
        w = self.pose
        return (w.distance < self.p.blind_finish_m and abs(w.axis_offset()) < self.p.blind_axis_m
                and abs(w.yaw_to_face) < math.radians(self.p.blind_yaw_deg))

    def _out(self, c: Command) -> Command:
        self._last_cmd = c
        return c

    def _finish(self, state: str, msg: str) -> Command:
        self.state, self.message, self._last_cmd = state, msg, Command()
        return Command()


def _unit(x: float, y: float) -> Tuple[float, float]:
    n = math.hypot(x, y) or 1.0
    return x / n, y / n


def _clip(v: float, m: float) -> float:
    return max(-m, min(m, v))


def _limit(v: float, vmax: float, vmin: float, deadband: float, err: float) -> float:
    """Ограничить скорость; вне мёртвой зоны — не меньше vmin (иначе моторы стоят)."""
    if abs(err) < deadband:
        return 0.0
    v = _clip(v, vmax)
    if 0 < abs(v) < vmin:
        v = math.copysign(vmin, v)
    return v


def draw_debug(bgr: np.ndarray, det: Optional[Detection], approach: WindowApproach) -> np.ndarray:
    """Кадр с рамкой окна и состоянием — для отладки в вебе."""
    img = bgr.copy()
    if det is not None:
        pts = det.corners.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, (0, 255, 0), 3)
        cx, cy = det.center
        cv2.circle(img, (int(cx), int(cy)), 6, (0, 255, 0), -1)
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
    text = f"{approach.state}"
    if approach.source:
        text += f" [{ {'камера': 'cam', 'одометрия': 'odom'}.get(approach.source, '')}]"
    if approach.pose is not None:
        p = approach.pose
        text += f"  d={p.distance:.2f}m  axis={p.axis_offset() * 100:+.0f}cm  yaw={math.degrees(p.yaw_to_face):+.0f}deg"
    cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
    cv2.putText(img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return img


def calibrate_hfov(det: Detection, width_px: int, distance_m: float, height_m: float) -> float:
    """Угол обзора обычной камеры по кадру с окном прямо перед камерой на известном расстоянии (до
    плоскости окна от объектива). Возвращает градусы."""
    tl, tr, br, bl = det.corners
    h_px = sum(det.edge_h) / 2 if det.edge_h else (abs(bl[1] - tl[1]) + abs(br[1] - tr[1])) / 2
    f = h_px * distance_m / height_m
    return math.degrees(2 * math.atan((width_px / 2) / f))


def calibrate_pitch(det: Detection, cam: CameraModel, g: WindowGeometry) -> float:
    """Поправка наклона камеры, градусы (прибавить к pitch_deg): по известной высоте низа рамки.
    Окно в кадре ниже, чем должно быть, — камера наклонена вверх сильнее, чем задано."""
    if g.bottom_m <= 0:
        raise ValueError("нужна высота низа рамки (window_bottom_m)")
    tl, tr, br, bl = det.corners
    v = (bl[1] + br[1]) / 2
    h_px = det.edge_h if det.edge_h else (abs(bl[1] - tl[1]), abs(br[1] - tr[1]))
    depth = g.height_m * cam.f / max(sum(h_px) / 2, 1.0)
    seen = math.atan2(cam.cy - v, cam.f)                       # под каким углом видим низ рамки
    true = math.atan2(g.bottom_m - cam.z_m, depth)             # под каким должны
    return math.degrees(true - seen)


def calibrate_lens_fov(det: Detection, cam: CameraModel, distance_m: float, height_m: float,
                       lens_fov_deg: float) -> float:
    """Угол обзора круга «рыбьего глаза» (модель ideal) по выпрямленному кадру с окном прямо перед
    камерой на известном расстоянии: если окно в кадре выше, чем должно быть, объектив «уже», чем
    задано. Точно для окна у центра кадра; повторить 2–3 раза, подставляя полученное."""
    tl, tr, br, bl = det.corners
    h_meas = sum(det.edge_h) / 2 if det.edge_h else (abs(bl[1] - tl[1]) + abs(br[1] - tr[1])) / 2
    h_exp = cam.f * height_m / distance_m
    return lens_fov_deg * h_exp / max(h_meas, 1.0)


__all__: List[str] = [
    "Detection", "DetectorParams", "detect_window", "detect_candidates", "red_mask", "CameraModel",
    "WindowGeometry", "WindowPose", "estimate_pose", "bottom_height", "check_window", "find_window",
    "ApproachParams", "Command", "WindowApproach", "draw_debug", "calibrate_hfov", "calibrate_lens_fov",
    "calibrate_pitch",
    "IDLE", "SEARCH", "APPROACH", "DONE", "FAILED",
]
