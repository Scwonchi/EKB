"""
Запись данных радара в CSV для обучения нейросети.

Работает с ld2410c_logger и с капсулой (capsule): капсуле сам отправляет команду rec.

Пример:
    python record.py --port COM5 --label pusto --seconds 120
    python record.py --port /dev/ttyACM0 --label pomeha --data data_capsule --note "куст на ветру"
    python record.py --port /dev/ttyACM0 --label nepodvizhen --seconds 60 --note "лежит 3м"

Файлы сохраняются в data/<метка>/<дата_время>.csv.
Каждая запись — одна ситуация (одна комната, одна поза, одно расстояние).
Ctrl+C останавливает запись досрочно, записанное сохраняется.

Установка: pip install pyserial
"""
import argparse
import csv
import datetime as dt
import os
import sys
import time

import serial

N_CH = 18
COLUMNS = [f"m{g}" for g in range(9)] + [f"s{g}" for g in range(9)]
SKIP_FIRST_S = 2.0          # первые секунды отбрасываются (переходные процессы после сброса ESP)


def parse(line: str):
    parts = line.strip().split(",")
    if len(parts) != N_CH:
        return None
    try:
        vals = [int(p) for p in parts]
    except ValueError:
        return None
    if any(v < 0 or v > 100 for v in vals):
        return None
    return vals


def main():
    ap = argparse.ArgumentParser(description="Запись данных LD2410C")
    ap.add_argument("--port", required=True, help="COM-порт ESP32 (COM5, /dev/ttyACM0 ...)")
    ap.add_argument("--label", required=True, help="метка класса, латиницей: pusto, dvizhetsya, nepodvizhen, pomeha")
    ap.add_argument("--seconds", type=float, default=60, help="длительность записи, с")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--data", default="data", help="папка с данными")
    ap.add_argument("--note", default="", help="заметка (условия записи), попадёт в журнал")
    ap.add_argument("--delay", type=float, default=3, help="отсчёт перед началом, с")
    args = ap.parse_args()

    if not args.label.replace("_", "").isalnum():
        sys.exit("Метка должна состоять из латинских букв, цифр и _")

    out_dir = os.path.join(args.data, args.label)
    os.makedirs(out_dir, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(out_dir, f"{stamp}.csv")

    ser = serial.Serial(args.port, args.baud, timeout=1)
    ser.dtr = False
    ser.rts = False
    time.sleep(0.5)
    ser.write(b"rec on\n")           # капсула (capsule.ino) начинает слать строки; ld2410c_logger шлёт их всегда
    print(f"Порт {args.port} открыт, жду данные радара...")

    # Ждём первые корректные строки
    t0 = time.time()
    while True:
        line = ser.readline().decode("ascii", errors="ignore")
        if parse(line):
            break
        if time.time() - t0 > 10:
            sys.exit("За 10 с не пришло ни одной строки данных. Прошит ли ld2410c_logger или capsule? Закрыт ли монитор порта?")

    for i in range(int(args.delay), 0, -1):
        print(f"Старт через {i}...", end="\r")
        time.sleep(1)
    ser.reset_input_buffer()
    print(f"\nЗАПИСЬ «{args.label}» на {args.seconds:.0f} с -> {path}")

    rows = []
    start = time.time()
    bad = 0
    try:
        while time.time() - start < args.seconds + SKIP_FIRST_S:
            line = ser.readline().decode("ascii", errors="ignore")
            vals = parse(line)
            if vals is None:
                if line.strip():
                    bad += 1
                continue
            if time.time() - start < SKIP_FIRST_S:
                continue
            rows.append(vals)
            el = time.time() - start - SKIP_FIRST_S
            if len(rows) % 10 == 0:
                bar = "#" * int(30 * el / args.seconds)
                print(f"[{bar:<30}] {el:5.0f} с, кадров {len(rows)}", end="\r")
    except KeyboardInterrupt:
        print("\nОстановлено вручную")
    finally:
        try:
            ser.write(b"rec off\n")
        except Exception:
            pass
        ser.close()

    if not rows:
        sys.exit("Ничего не записано")

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerows(rows)

    dur = len(rows) / 10
    with open(os.path.join(args.data, "journal.csv"), "a", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow([stamp, args.label, path, len(rows), f"{dur:.0f}", args.note])

    print(f"\nСохранено {len(rows)} кадров (~{dur:.0f} с){', битых строк ' + str(bad) if bad else ''}")
    if dur < 15:
        print("Внимание: запись короче 15 с почти не даст окон для обучения (окно 10 с)")


if __name__ == "__main__":
    main()
