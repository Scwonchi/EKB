"""Проверка распознавания окна на снимках с площадки — без ровера и ROS.

    ros2 run rover_window_approach window_check кадры/*.jpg --hfov 70 --pitch 30 --camera-z 0.45 \\
        --width 0.85 --height 0.7 --bottom 0.85 --out размечено/
    python3 -m rover_window_approach.check кадры/*.jpg --lens ideal --fov 190     # «рыбий глаз» ровера

Снимки — как их отдаёт камера (веб-камера: http://<вторая Pi>:8080/frame.jpg). Для каждого кадра:
найдено ли окно, расстояние, смещение оси, угол; с --out — выпрямленные кадры с рамкой. Сводка —
доля кадров с окном. Имя файла с «noWindow»/«bez» — кадр без окна: находка на нём считается
ложной (так считаются полнота и ложные срабатывания для таблицы метрик).
"""
import argparse
import math
import os
import sys

import cv2

from .lens import LensParams, Rectifier
from .window_core import CameraModel, DetectorParams, WindowApproach, WindowGeometry, draw_debug, find_window


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('images', nargs='+')
    ap.add_argument('--lens', default='pinhole', help='pinhole (веб-камера) | ideal | calibrated')
    ap.add_argument('--hfov', type=float, default=70.0, help='pinhole: угол обзора по горизонтали, °')
    ap.add_argument('--pitch', type=float, default=30.0, help='наклон камеры вверх, °')
    ap.add_argument('--fov', type=float, default=180.0, help='ideal: угол обзора круга, °')
    ap.add_argument('--calib', default='', help='calibrated: файл lens_calib')
    ap.add_argument('--camera-x', type=float, default=0.05)
    ap.add_argument('--camera-z', type=float, default=0.45, help='высота объектива, м')
    ap.add_argument('--width', type=float, default=0.85, help='внешняя ширина рамки, м')
    ap.add_argument('--height', type=float, default=0.7, help='внешняя высота рамки, м')
    ap.add_argument('--bottom', type=float, default=0.0, help='низ рамки над полом, м (0 — не проверять точно)')
    ap.add_argument('--out', help='папка для размеченных кадров')
    a = ap.parse_args(argv)
    if a.out:
        os.makedirs(a.out, exist_ok=True)
    rect = Rectifier(LensParams(model=a.lens, hfov_deg=a.hfov, pitch_deg=a.pitch, fov_deg=a.fov, calib_file=a.calib))
    geom = WindowGeometry(width_m=a.width, height_m=a.height, bottom_m=a.bottom)
    tp = fn = fp = tn = 0
    for path in a.images:
        src = cv2.imread(path)
        if src is None:
            print(f'{path}: не читается')
            continue
        neg = any(s in os.path.basename(path).lower() for s in ('nowindow', 'bez'))
        img = rect.rectify(src)
        cam = CameraModel.rectified(rect, (src.shape[1], src.shape[0]), a.camera_x, 0.0, a.camera_z)
        det, p = find_window(img, cam, geom, DetectorParams())
        line = f'{os.path.basename(path)}: '
        if p is None:
            line += 'окна нет' if det is None else 'рамка обрезана краем кадра'
        else:
            line += (f'окно, до плоскости {p.distance:.2f} м, ось ровера смещена на {p.axis_offset() * 100:+.0f} см, '
                     f'повернуться на {math.degrees(p.yaw_to_face):+.0f}°')
        print(line)
        found = p is not None
        if neg:
            fp += found
            tn += not found
        else:
            tp += found
            fn += not found
        if a.out:
            cv2.imwrite(os.path.join(a.out, os.path.basename(path)), draw_debug(img, det, WindowApproach()))
    pos, negn = tp + fn, fp + tn
    print(f'\nКадров с окном: {pos}, найдено {tp} ({100 * tp / max(pos, 1):.0f}%); '
          f'кадров без окна: {negn}, ложных находок {fp} ({100 * fp / max(negn, 1):.0f}%)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
