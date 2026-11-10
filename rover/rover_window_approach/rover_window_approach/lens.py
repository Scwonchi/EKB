"""Кадр камеры → ровная камера-обскура.

Два варианта камеры:
* веб-камера на второй Raspberry Pi (``model: pinhole``) — обычный объектив; кадр только
  поворачивается так, будто камера смотрит горизонтально (если она наклонена вверх);
* штатная камера ровера с «рыбьим глазом» (``ideal`` / ``calibrated``) — ниже.

У штатной камеры ровера круглый кадр с сильной бочкой: прямые края рамки в нём изогнуты, а размер
предмета в пикселях зависит от того, в каком месте кадра он стоит. Расчёт расстояния по высоте
рамки верен только для камеры-обскуры, поэтому кадр сначала выпрямляется (``Rectifier``) в
«виртуальную» обычную камеру:

* она смотрит строго горизонтально (наклон камеры на ровере ``pitch_deg`` вычитается) — тогда
  вертикальные края окна в кадре вертикальны и расчёт по их высоте точен;
* главная точка сдвинута вниз (``cy_frac``): окно стоит высоко, а камера низко, поэтому верхней
  части обзора нужно больше, чем нижней.

Модель объектива:
* ``ideal`` — идеальный равноугольный «рыбий глаз» (r = f·θ) по кругу изображения: центр, радиус
  и угол обзора круга (``fov_deg``). Без калибровки, точность — несколько процентов;
* ``calibrated`` — по шахматной доске (``lens_calib``): файл с K и D модели cv2.fisheye;
* ``pinhole`` — обычная камера (веб-камера): угол обзора ``hfov_deg``, выпрямляется только
  наклон; без наклона кадр не меняется.

Выпрямление — одна таблица ``cv2.remap``, считается один раз на размер кадра (1280×1024 → 800×600
на Raspberry Pi — несколько миллисекунд).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class LensParams:
    model: str = "pinhole"         # pinhole (веб-камера) | ideal | calibrated («рыбий глаз»)
    fov_deg: float = 180.0         # ideal: угол обзора по диаметру круга изображения
    circle_cx: float = 0.0         # ideal: центр круга, px исходного кадра (0 — середина кадра)
    circle_cy: float = 0.0
    circle_r: float = 0.0          # ideal: радиус круга, px (0 — половина меньшей стороны кадра)
    calib_file: str = ""           # calibrated: файл lens_calib (K, D, размер кадра)
    pitch_deg: float = 0.0         # камера смотрит вверх на столько градусов (вниз — минус)
    hfov_deg: float = 70.0         # pinhole: угол обзора по горизонтали
    out_width: int = 800           # «рыбий глаз»: выпрямленный кадр
    out_height: int = 600
    out_hfov_deg: float = 110.0    # «рыбий глаз»: угол обзора выпрямленного кадра по горизонтали
    out_cy_frac: float = 0.75      # «рыбий глаз»: главная точка на этой доле высоты кадра (0,5 — середина)

    @property
    def is_pinhole(self) -> bool:
        return self.model in ("pinhole", "none")


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], np.float64)


def ideal_k(width: int, height: int, p: LensParams) -> Tuple[np.ndarray, np.ndarray]:
    """K и D идеального равноугольного объектива по кругу изображения."""
    cx = p.circle_cx or (width - 1) / 2
    cy = p.circle_cy or (height - 1) / 2
    r = p.circle_r or min(width, height) / 2
    f = r / math.radians(p.fov_deg / 2)
    return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], np.float64), np.zeros((4, 1), np.float64)


def load_calibration(path: str) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int]]:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"нет файла калибровки {path}")
    K = fs.getNode("K").mat()
    D = fs.getNode("D").mat().reshape(4, 1)
    size = (int(fs.getNode("image_width").real()), int(fs.getNode("image_height").real()))
    fs.release()
    return K.astype(np.float64), D.astype(np.float64), size


def save_calibration(path: str, K: np.ndarray, D: np.ndarray, size: Tuple[int, int], rms: float) -> None:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("model", "opencv_fisheye")
    fs.write("image_width", int(size[0]))
    fs.write("image_height", int(size[1]))
    fs.write("K", K)
    fs.write("D", D.reshape(4, 1))
    fs.write("rms_px", float(rms))
    fs.release()


class Rectifier:
    """Кадр камеры ровера → выпрямленный кадр ровной виртуальной камеры-обскуры."""

    def __init__(self, p: Optional[LensParams] = None):
        self.p = p or LensParams()
        self._size: Optional[Tuple[int, int]] = None
        self._maps = None
        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None

    # Параметры выпрямленной камеры
    @property
    def f(self) -> float:
        return (self.p.out_width / 2) / math.tan(math.radians(self.p.out_hfov_deg) / 2)

    def out_camera(self, src_size: Tuple[int, int]):
        """(width, height, f, cx, cy) камеры, в которой получен выпрямленный кадр."""
        if self.p.is_pinhole:
            # Тот же кадр, но ровная камера: кадр по высоте растягивается так, чтобы в него вошло
            # всё, что видела наклонённая камера (верх и низ её обзора)
            w, h = src_size
            f = (w / 2) / math.tan(math.radians(self.p.hfov_deg) / 2)
            half_v = math.atan((h / 2) / f)
            pitch = math.radians(self.p.pitch_deg)
            if abs(pitch) < 1e-9:
                return w, h, f, (w - 1) / 2, (h - 1) / 2
            up = f * math.tan(min(pitch + half_v, math.radians(80)))
            down = f * math.tan(min(half_v - pitch, math.radians(80)))
            return w, int(math.ceil(up + down)) + 2, f, (w - 1) / 2, up + 1
        w, h = self.p.out_width, self.p.out_height
        return w, h, self.f, (w - 1) / 2, self.p.out_cy_frac * h

    def source_k(self, src_size: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray]:
        w, h = src_size
        if self.p.model == "calibrated":
            K, D, (cw, ch) = load_calibration(self.p.calib_file)
            if (cw, ch) != (w, h):                       # калибровали на другом размере кадра
                sx, sy = w / cw, h / ch
                K = K.copy()
                K[0, 0] *= sx
                K[0, 2] *= sx
                K[1, 1] *= sy
                K[1, 2] *= sy
            return K, D
        return ideal_k(w, h, self.p)

    def _build(self, src_size: Tuple[int, int]) -> None:
        w, h, f, cx, cy = self.out_camera(src_size)
        P = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], np.float64)
        R = _rot_x(self.p.pitch_deg)
        if self.p.is_pinhole:
            sw, sh = src_size
            fs = (sw / 2) / math.tan(math.radians(self.p.hfov_deg) / 2)
            self.K = np.array([[fs, 0, (sw - 1) / 2], [0, fs, (sh - 1) / 2], [0, 0, 1]], np.float64)
            self.D = np.zeros((4, 1))
            self._H = self.K @ R.T @ np.linalg.inv(P)           # пиксель ровной камеры → исходный
            self._out = (w, h)
        else:
            self.K, self.D = self.source_k(src_size)
            self._maps = cv2.fisheye.initUndistortRectifyMap(self.K, self.D, R, P, (w, h), cv2.CV_16SC2)
        self._size = src_size

    def rectify(self, bgr: np.ndarray) -> np.ndarray:
        if self.p.is_pinhole and abs(self.p.pitch_deg) < 1e-6:
            return bgr
        size = (bgr.shape[1], bgr.shape[0])
        if self._size != size:
            self._build(size)
        if self.p.is_pinhole:
            return cv2.warpPerspective(bgr, self._H, self._out, flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        m1, m2 = self._maps
        return cv2.remap(bgr, m1, m2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


def find_image_circle(bgr: np.ndarray, threshold: int = 25) -> Optional[Tuple[float, float, float]]:
    """Круг изображения «рыбьего глаза» (cx, cy, r) по тёмным углам кадра; None — кадр без чёрных углов."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    h, w = gray.shape
    m = (gray > threshold).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    cs, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    # Окружность по точкам границы, кроме тех, что лежат на краю кадра (круг может быть обрезан)
    pts = c.reshape(-1, 2).astype(np.float64)
    inside = (pts[:, 0] > 2) & (pts[:, 0] < w - 3) & (pts[:, 1] > 2) & (pts[:, 1] < h - 3)
    pts = pts[inside]
    if len(pts) < 30:
        return None
    A = np.c_[2 * pts[:, 0], 2 * pts[:, 1], np.ones(len(pts))]
    b = (pts ** 2).sum(axis=1)
    (cx, cy, k), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = math.sqrt(max(k + cx * cx + cy * cy, 0))
    return float(cx), float(cy), float(r)


__all__ = ["LensParams", "Rectifier", "ideal_k", "load_calibration", "save_calibration", "find_image_circle"]
