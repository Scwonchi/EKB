#!/usr/bin/env python3
"""Сторона ровера для стендовой проверки микродрона — без ROS, на ноутбуке или на ровере.

Делает то, что в миссии будет делать нода ровера: рассылает UDP-маяк, принимает подключение
микродрона, отправляет план и разрешение на взлёт, печатает события и тревоги, пересчитывает
координаты людей в систему полигона и пишет журнал с отметками времени.

    python3 rover_stub.py --forward 0.8 --height 1.2 --rover-x 2.1 --rover-y 1.4 --x-side right

Клавиши (и Enter): p — отправить план, g — разрешить взлёт, a — прервать (посадка), q — выход.
С --snap при тревоге сохраняет кадр из видеопотока микродрона (нужен OpenCV с поддержкой RTSP).
"""
import argparse
import json
import math
import os
import socket
import sys
import threading
import time


def to_polygon(f, l, a):
    """Точка в осях ровера от площадки (вперёд, влево) -> координаты полигона (X, Y).
    Начало полигона — точка старта ровера, Y — по направлению ровера на старте."""
    F, L = a.pad_x + f, a.pad_y + l
    th = math.radians(a.rover_heading)
    fwd = F * math.cos(th) - L * math.sin(th)
    left = F * math.sin(th) + L * math.cos(th)
    x = a.rover_x + (left if a.x_side == "left" else -left)
    return x, a.rover_y + fwd


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--forward", type=float, default=0.8, help="от площадки до плоскости окна, м")
    ap.add_argument("--left", type=float, default=0.0, help="центр окна левее оси ровера, м")
    ap.add_argument("--height", type=float, default=1.2, help="высота центра окна над полом, м")
    ap.add_argument("--width", type=float, default=0.85)
    ap.add_argument("--height-size", type=float, default=0.70)
    ap.add_argument("--scan-height", type=float, default=1.0)
    ap.add_argument("--points", help='точки обзора в осях ровера: "1.8,0;1.8,1.2;1.8,-1.2"')
    ap.add_argument("--rover-x", type=float, default=0.0, help="ровер в системе полигона, м")
    ap.add_argument("--rover-y", type=float, default=0.0)
    ap.add_argument("--rover-heading", type=float, default=0.0, help="курс ровера от оси Y полигона, ° (влево +)")
    ap.add_argument("--x-side", choices=("left", "right"), default="right", help="ось X полигона — слева или справа от ровера на старте")
    ap.add_argument("--pad-x", type=float, default=0.0, help="площадка впереди центра ровера, м")
    ap.add_argument("--pad-y", type=float, default=0.0)
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--beacon-port", type=int, default=8771)
    ap.add_argument("--log", default="mission_log.jsonl")
    ap.add_argument("--snap", action="store_true", help="сохранять кадр RTSP при тревоге")
    ap.add_argument("--auto", action="store_true", help="план и взлёт без клавиш (только для имитатора!)")
    a = ap.parse_args(argv)

    plan = {"t": "plan", "window": {"forward": a.forward, "left": a.left, "height": a.height,
                                    "width": a.width, "height_size": a.height_size},
            "search": {"height": a.scan_height}}
    if a.points:
        plan["search"]["points"] = [[float(v) for v in p.split(",")] for p in a.points.split(";")]

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", a.port))
    srv.listen(1)
    state = {"conn": None, "run": True, "peer": None, "done": None}
    logf = open(a.log, "a")

    def record(direction, m):
        logf.write(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%S") + ".%03d" % (time.time() % 1 * 1000),
                               "dir": direction, "msg": m}, ensure_ascii=False) + "\n")
        logf.flush()

    def send(m):
        c = state["conn"]
        if c is None:
            print("микродрон не подключён")
            return
        try:
            c.sendall((json.dumps(m, ensure_ascii=False) + "\n").encode("utf-8"))
            if m.get("t") not in ("ping", "pong"):
                record("rover->whoop", m)
        except OSError:
            pass

    def beacon():
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        data = json.dumps({"ekb": "rover", "port": a.port}).encode()
        while state["run"]:
            for dst in ("255.255.255.255", "127.0.0.1"):
                try:
                    u.sendto(data, (dst, a.beacon_port))
                except OSError:
                    pass
            time.sleep(1.0)

    def snapshot(m):
        try:
            import cv2
            url = "rtsp://%s%s" % (state["peer"], state.get("rtsp", "/h264"))
            cap = cv2.VideoCapture(url)
            ok, img = cap.read()
            cap.release()
            if ok:
                name = "alarm_%d_%s.jpg" % (m["id"], time.strftime("%H%M%S"))
                cv2.imwrite(name, img)
                print("   снимок: %s" % name)
            else:
                print("   снимок не получен (%s)" % url)
        except Exception as e:                          # noqa: BLE001
            print("   снимок не получен: %r" % (e,))

    def handle(m):
        t = m.get("t")
        if t == "ping":
            send({"t": "pong"})
            return
        if t in ("pong", "state"):
            if t == "state":
                state["last_state"] = m
            return
        record("whoop->rover", m)
        if t == "hello":
            sc = m.get("selfcheck", {})
            state["rtsp"] = m.get("rtsp_path", "/h264")
            print("микродрон на связи, версия %s; предполётная проверка: %s %s"
                  % (m.get("version"), {True: "пройдена", False: "НЕ ПРОЙДЕНА", None: "не выполнена"}.get(sc.get("ok")),
                     sc.get("items", sc.get("note", ""))))
            if a.auto and not state.get("sent_plan"):
                state["sent_plan"] = True
                send(plan)
        elif t == "plan_ok":
            print("план принят микродроном. g — разрешить взлёт")
            if a.auto:
                send({"t": "go"})
        elif t == "plan_bad":
            print("план отклонён: %s" % m.get("reason"))
        elif t == "event":
            print("  %s %s" % (m.get("name"), dict((k, v) for k, v in m.items() if k not in ("t", "name"))))
        elif t == "detection":
            x, y = to_polygon(m["f"], m["l"], a)
            print("%s №%d: пеленг %+.0f°, дистанция %s%.1f м; полигон X=%.2f Y=%.2f"
                  % ("уточнение" if m.get("update") else "ТРЕВОГА", m["id"], m["bearing_deg"],
                     "~" if m.get("range_approx") else "", m["range_m"], x, y))
            if a.snap and not m.get("update"):
                threading.Thread(target=snapshot, args=(m,), daemon=True).start()
        elif t == "victims":
            print("ИТОГ: людей %d" % m["count"])
            out = []
            for v in m["victims"]:
                x, y = to_polygon(v["f"], v["l"], a)
                out.append({"id": v["id"], "x": round(x, 2), "y": round(y, 2), "approx": v["approx"]})
                print("   №%d: X=%.2f Y=%.2f %s" % (v["id"], x, y, "(грубо: виден из одной точки)" if v["approx"] else ""))
            record("result", {"polygon": out})
        elif t == "done":
            print("миссия %s %s" % ("выполнена" if m.get("ok") else "ПРЕРВАНА:", m.get("reason", "")))
            state["done"] = m

    def serve():
        while state["run"]:
            srv.settimeout(1.0)
            try:
                c, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            state["conn"], state["peer"] = c, addr[0]
            print("подключился микродрон %s. p — отправить план" % addr[0])
            buf = b""
            c.settimeout(1.0)
            last_ping = 0.0
            while state["run"]:
                if time.time() - last_ping > 0.5:
                    last_ping = time.time()
                    send({"t": "ping"})
                try:
                    chunk = c.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    try:
                        handle(json.loads(line.decode("utf-8")))
                    except ValueError:
                        pass
            state["conn"] = None
            print("микродрон отключился")

    threading.Thread(target=beacon, daemon=True).start()
    threading.Thread(target=serve, daemon=True).start()
    print("жду микродрон на порту %d (маяк UDP %d). Журнал: %s" % (a.port, a.beacon_port, a.log))
    try:
        if a.auto:
            while state["done"] is None:
                time.sleep(0.1)
        else:
            for line in sys.stdin:
                k = line.strip().lower()
                if k == "p":
                    send(plan)
                elif k == "g":
                    send({"t": "go"})
                elif k == "a":
                    send({"t": "abort"})
                elif k == "q":
                    break
    except KeyboardInterrupt:
        pass
    send({"t": "abort"}) if state["done"] is None and not a.auto else None
    state["run"] = False
    time.sleep(0.2)
    srv.close()
    return 0 if (state["done"] or {}).get("ok", True) else 1


if __name__ == "__main__":
    sys.exit(main())
