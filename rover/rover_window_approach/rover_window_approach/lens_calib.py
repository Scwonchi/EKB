"""Калибровка объектива «рыбий глаз» камеры ровера.

1. Круг изображения (для модели ideal) — по одному любому кадру:

       ros2 run rover_window_approach lens_calib --circle кадр.jpg

   Печатает lens_circle_cx / cy / r для config/window_approach.yaml.

2. Шахматная доска (модель calibrated, точнее) — 12–20 снимков доски с разных сторон, доска
   и у центра, и у края круга:

       ros2 run rover_window_approach lens_calib доска/*.jpg --cols 9 --rows 6 --square 0.025 \\
           --out ~/rover_fisheye.yaml

   cols × rows — число ВНУТРЕННИХ углов доски, square — сторона клетки, м. Файл — в
   lens_calib_file, lens_model: calibrated. Ошибка репроекции (rms) меньше 1 px — хорошо.

3. Проверка — выпрямленный кадр: прямые на нём должны быть прямыми:

       ros2 run rover_window_approach lens_calib --preview кадр.jpg --model ideal --fov 190 --out выпрямлено.jpg

Снимки с ровера: ``ros2 service call /get_frame rover_interfaces/srv/GetFrame`` или кадры из веб-интерфейса.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .lens import LensParams, Rectifier, find_image_circle, save_calibration


def _flag(name: str) -> int:
    """Флаги калибровки: в OpenCV 4 — cv2.fisheye.CALIB_*, в OpenCV 5 — cv2.CALIB_*."""
    return int(getattr(cv2.fisheye, name, None) or getattr(cv2, name))


def calibrate(images: Sequence[np.ndarray], cols: int, rows: int, square: float
              ) -> Tuple[float, np.ndarray, np.ndarray, Tuple[int, int], List[int]]:
    """(rms, K, D, размер кадра, номера снимков, где доска найдена)."""
    objp = np.zeros((1, cols * rows, 3), np.float64)
    objp[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    obj, img_pts, used = [], [], []
    size = None
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    for i, im in enumerate(images):
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
        size = (gray.shape[1], gray.shape[0])
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows),
                                                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            continue
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
        obj.append(objp.copy())
        img_pts.append(corners.reshape(1, -1, 2).astype(np.float64))
        used.append(i)
    if len(obj) < 4:
        raise RuntimeError(f"доска найдена только на {len(obj)} снимках, нужно хотя бы 4 (лучше 12+)")
    # Начальное приближение — по кругу изображения (объектив ~180°): без него оптимизация у
    # широких объективов не сходится
    circle = find_image_circle(images[0])
    cx, cy, r = circle if circle else ((size[0] - 1) / 2, (size[1] - 1) / 2, min(size) / 2)
    f0 = r / (math.pi / 2)
    K0 = np.array([[f0, 0, cx], [0, f0, cy], [0, 0, 1]], np.float64)
    K = K0.copy()
    D = np.zeros((4, 1))
    flags = _flag('CALIB_USE_INTRINSIC_GUESS') | _flag('CALIB_RECOMPUTE_EXTRINSIC') | _flag('CALIB_FIX_SKEW')
    # Снимок, на котором оптимизация не сходится, выкидываем и пробуем снова
    while True:
        try:
            rms, K, D, _, _ = cv2.fisheye.calibrate(obj, img_pts, size, K, D, flags=flags | _flag('CALIB_CHECK_COND'),
                                                    criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-7))
            break
        except cv2.error as e:
            K, D = K0.copy(), np.zeros((4, 1))
            msg = str(e)
            m = re.search(r"input array (\d+)", msg)
            if not m or len(obj) <= 4:
                rms, K, D, _, _ = cv2.fisheye.calibrate(obj, img_pts, size, K, D, flags=flags,
                                                        criteria=(cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 100, 1e-7))
                break
            k = int(m.group(1))
            obj.pop(k)
            img_pts.pop(k)
            used.pop(k)
    return float(rms), K, D, size, used


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="+")
    ap.add_argument("--circle", action="store_true", help="найти круг изображения по кадру")
    ap.add_argument("--preview", action="store_true", help="выпрямить кадр с заданной моделью")
    ap.add_argument("--cols", type=int, default=9, help="внутренних углов доски по горизонтали")
    ap.add_argument("--rows", type=int, default=6, help="внутренних углов доски по вертикали")
    ap.add_argument("--square", type=float, default=0.025, help="сторона клетки, м")
    ap.add_argument("--model", default="ideal", help="для --preview: ideal | calibrated")
    ap.add_argument("--fov", type=float, default=180.0, help="для --preview ideal: угол обзора круга, °")
    ap.add_argument("--calib", default="", help="для --preview calibrated: файл калибровки")
    ap.add_argument("--pitch", type=float, default=0.0, help="для --preview: наклон камеры вверх, °")
    ap.add_argument("--out", help="файл калибровки (.yaml) или выпрямленного кадра (--preview)")
    a = ap.parse_args(argv)
    imgs = [cv2.imread(p) for p in a.images]
    if any(i is None for i in imgs):
        print("не читается: " + ", ".join(p for p, i in zip(a.images, imgs) if i is None))
        return 2
    if a.circle:
        for p, im in zip(a.images, imgs):
            c = find_image_circle(im)
            if c is None:
                print(f"{p}: круг не найден (кадр без чёрных углов — обычный объектив или круг больше кадра)")
            else:
                print(f"{p}: кадр {im.shape[1]}×{im.shape[0]}\n  lens_circle_cx: {c[0]:.1f}\n"
                      f"  lens_circle_cy: {c[1]:.1f}\n  lens_circle_r: {c[2]:.1f}")
        return 0
    if a.preview:
        c = find_image_circle(imgs[0]) or (0.0, 0.0, 0.0)
        r = Rectifier(LensParams(model=a.model, fov_deg=a.fov, circle_cx=c[0], circle_cy=c[1], circle_r=c[2],
                                 calib_file=a.calib, pitch_deg=a.pitch))
        out = r.rectify(imgs[0])
        h = out.shape[0]
        for y in range(0, h, h // 12):                       # сетка: прямые на кадре должны идти вдоль неё
            cv2.line(out, (0, y), (out.shape[1], y), (0, 255, 255), 1)
        cv2.imwrite(a.out or "rectified.jpg", out)
        print(f"выпрямленный кадр: {a.out or 'rectified.jpg'}")
        return 0
    rms, K, D, size, used = calibrate(imgs, a.cols, a.rows, a.square)
    print(f"доска найдена на {len(used)} из {len(imgs)} снимков, ошибка репроекции {rms:.2f} px")
    print(f"K =\n{K}\nD = {D.ravel()}")
    out = a.out or "fisheye_calib.yaml"
    save_calibration(out, K, D, size, rms)
    print(f"записано в {out}: lens_model: calibrated, lens_calib_file: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
