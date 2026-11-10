"""
Запись данных для нейросети треков с разведчика (rescue_scout) по USB.

    python tools/record_tracks.py --port /dev/ttyACM0 --label pusto --seconds 120
    python tools/record_tracks.py --port /dev/ttyACM0 --label chelovek --zone 2-3.5 --seconds 90 --note "сидит"
    python tools/record_tracks.py --port COM5 --label chelovek --zone 1.5-3 --angle -20..20

Программа сама включает на устройстве режим записи (команда rec) и выключает его в конце.
Файл: data_tracks/<метка>/<дата_время>.csv. Ctrl+C — остановить досрочно, записанное сохраняется.

Метки:
  pusto*     — в зоне радара людей нет (комната, шторы, вентилятор, стены рядом — всё,
               что даёт ложные точки). Все треки этой записи — ложные.
  остальные  — человек есть. --zone (дальность, м) и --angle (угол, °, как в команде tracks)
               говорят, где он: треки в зоне — человек, вне зоны — ложные (призраки, шум).
               Зону берите с запасом ±0,5 м; для идущего человека — весь его путь.

Монитор порта Arduino IDE на это время закройте. Установка: pip install pyserial
"""
import argparse
import datetime as dt
import os
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--label", required=True, help="pusto, chelovek, chelovek_lezhit ...")
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--zone", help="дальность человека, м: 2-3.5")
    ap.add_argument("--angle", help="угол человека, °: -30..30")
    ap.add_argument("--note", default="")
    ap.add_argument("--data", default=os.path.join(ROOT, "data_tracks"))
    a = ap.parse_args()
    if not a.label.startswith("pusto") and not a.zone:
        print("Подсказка: без --zone в записи с человеком сеть учится хуже (призраки не размечены)")

    d = os.path.join(a.data, a.label)
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, dt.datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
    s = serial.Serial(a.port, 115200, timeout=0.2)
    s.dtr = False
    s.rts = False
    time.sleep(1.0)
    s.reset_input_buffer()
    s.write(b"rec on\n")

    nR = nF = 0
    t0 = time.time()
    last = 0
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"# source=device label={a.label}")
        if a.zone:
            f.write(f" zone={a.zone}")
        if a.angle:
            f.write(f" angle={a.angle}")
        f.write(f" date={dt.datetime.now().isoformat(timespec='seconds')}\n")
        if a.note:
            f.write(f"# note: {a.note}\n")
        try:
            while time.time() - t0 < a.seconds:
                line = s.readline().decode("utf-8", "replace").strip()
                if line.startswith("R,"):
                    nR += 1
                elif line.startswith("F,"):
                    nF += 1
                else:
                    continue
                f.write(line + "\n")
                if time.time() - last > 1:
                    last = time.time()
                    print(f"\r  {time.time() - t0:5.0f} с: кадров радара {nR}, строк признаков {nF}   ", end="", flush=True)
        except KeyboardInterrupt:
            pass
    s.write(b"rec off\n")
    s.close()
    print(f"\nЗаписано: {path}")
    if nR == 0:
        print("Нет строк R: та ли прошивка (команда rec есть с этой версии), закрыт ли монитор порта, идут ли данные радара (diag)")
        sys.exit(1)


if __name__ == "__main__":
    main()
