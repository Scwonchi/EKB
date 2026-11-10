#!/usr/bin/env python3
"""Проверка камеры и детектора микродрона на столе, без полёта (винты снять).

Документация микродрона не даёт размер кадра, в пикселях которого детектор отдаёт рамки, и
угол обзора камеры. От них зависят пеленги на людей и прицеливание в окно, поэтому их измеряют.

    python3 cam_check.py                      # печатать рамки и углы; двигайте предмет к краям кадра
    python3 cam_check.py --fov --label window --width 0.85 --distance 1.5
                                              # угол обзора: окно прямо перед камерой на 1,5 м

Размер кадра: поднесите предмет так, чтобы рамка упёрлась в правый и нижний край. Наибольшие
x2 и y2, которые напечатает программа, — это frame_w и frame_h для config.json.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geometry import Camera                                # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--config", default=os.path.join(here, "config.json"))
    ap.add_argument("--fov", action="store_true", help="посчитать угол обзора по предмету известной ширины")
    ap.add_argument("--label", default="", help="класс предмета (пусто — любой)")
    ap.add_argument("--width", type=float, default=0.85, help="ширина предмета, м")
    ap.add_argument("--distance", type=float, default=1.5, help="от объектива до предмета, м")
    ap.add_argument("--seconds", type=float, default=0.0, help="работать столько секунд (0 — до Ctrl+C)")
    ap.add_argument("--fake", action="store_true")
    a = ap.parse_args(argv)
    cfg = {}
    if os.path.exists(a.config):
        cfg = json.load(open(a.config))
    cam = Camera(cfg.get("frame_w", 640), cfg.get("frame_h", 640), cfg.get("hfov_deg", 90.0), cfg.get("cam_pitch_deg", 0.0))
    if a.fake:
        from clock import Clock
        from fake_offboard import FakeOffboard
        drone = FakeOffboard(Clock())
    else:
        from simple_offboard_py import SimpleOffboard
        drone = SimpleOffboard()
    mx = my = 0.0
    widths = []
    t0 = time.time()
    print("кадр в настройках %dx%d, угол обзора %.0f°. Ctrl+C — выход" % (cam.w, cam.h, cfg.get("hfov_deg", 90.0)))
    try:
        while not a.seconds or time.time() - t0 < a.seconds:
            for det in drone.get_detections():
                if a.label and det.label != a.label:
                    continue
                mx, my = max(mx, det.x2), max(my, det.y2)
                b = cam.box(det)
                print("%-10s %.2f  рамка (%4.0f,%4.0f)-(%4.0f,%4.0f)  азимут %+5.1f°  угол места %+5.1f°  наибольшие x2=%.0f y2=%.0f"
                      % (det.label, det.score, det.x1, det.y1, det.x2, det.y2,
                         b["az"] * 57.2958, b["el"] * 57.2958, mx, my))
                if a.fov and not b["cut"]:
                    widths.append(det.x2 - det.x1)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass
    if mx > cam.w + 2 or my > cam.h + 2:
        print("\nрамки выходят за кадр из настроек: frame_w не меньше %.0f, frame_h не меньше %.0f" % (mx, my))
    if a.fov:
        if len(widths) < 5:
            print("\nпредмет целиком в кадре виден меньше 5 раз — угол обзора не посчитан")
        else:
            widths.sort()
            w = widths[len(widths) // 2]
            print("\nширина предмета в кадре %.0f px -> hfov_deg: %.1f (при frame_w = %d)"
                  % (w, cam.hfov_from_object(w, a.width, a.distance), cam.w))
    return 0


if __name__ == "__main__":
    sys.exit(main())
