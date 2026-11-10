#!/usr/bin/env python3
"""Программа управления микродроном «Сверх» для кейса 3 «Операция – спасение».

Запускается на бортовом компьютере микродрона. Ждёт от ровера план и разрешение на взлёт,
взлетает с площадки ровера, пролетает окно, ищет людей и сообщает роверу тревоги и координаты.
Полётом управляет модуль simple_offboard_py из образа микродрона.

    python3 whoop_agent.py                       # на борту: ждать ровер
    python3 whoop_agent.py --plan plan.json      # на борту без ровера: план из файла
    python3 whoop_agent.py --fake --plan plan.json --speed 20   # на компьютере, с имитатором

Перед автономным полётом — как требует документация микродрона: пульт в руках, переключатель SB
готов к перехвату в режим Position, SD (Kill Switch) — аварийное отключение моторов.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from clock import Clock                                   # noqa: E402
from link import Link                                     # noqa: E402
from mission import Abort, Mission, NullLink              # noqa: E402

VERSION = "1.0"


def log(s):
    sys.stdout.write("[%s] %s\n" % (time.strftime("%H:%M:%S"), s))
    sys.stdout.flush()


def load_config(path):
    cfg = {}
    if path and os.path.exists(path):
        with open(path, "r") as f:
            cfg = json.load(f)
    return dict((k, v) for k, v in cfg.items() if not k.startswith("_"))


def selfcheck(cmd):
    """Предполётная проверка `drone selfcheck`. Возвращает {"ok": True | False | None, ...}.
    По документации: строки sys_status быть не должно, gyro, accelerometer и rc receiver — PASS.
    None — проверку выполнить не удалось (команды нет)."""
    if not cmd:
        return {"ok": None, "note": "проверка отключена"}
    try:
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        t0 = time.time()
        while p.poll() is None and time.time() - t0 < 30:
            time.sleep(0.2)
        if p.poll() is None:
            p.kill()
            return {"ok": False, "note": "selfcheck не завершился за 30 с"}
        out = p.stdout.read().decode("utf-8", "replace")
    except OSError as e:
        return {"ok": None, "note": "selfcheck не запустился: %s" % (e,)}
    if p.returncode == 127:
        return {"ok": None, "note": "команда selfcheck не найдена"}
    items = {}
    for line in out.splitlines():
        low = line.lower()
        for key in ("sys_status", "gyro", "accelerometer", "magnetometer", "gps", "vision_position",
                    "vision position", "xy position control", "rc receiver"):
            if key in low:
                st = "pass" if "pass" in low else "fail" if "fail" in low else "skip" if "skip" in low else "?"
                items[key.replace(" ", "_")] = st
    need = ("gyro", "accelerometer", "rc_receiver")
    ok = "sys_status" not in items and all(items.get(k) == "pass" for k in need)
    return {"ok": ok, "items": items, "raw": out[-600:]}


def ensure_detector(cfg):
    """Запустить детектор YOLO, если он не работает (по документации микродрона)."""
    cmd = cfg.get("detector_cmd")
    if not cfg.get("start_detector") or not cmd:
        return
    name = os.path.basename(cmd.split()[0])
    try:
        if subprocess.call(["pidof", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            return
        # распознавание лиц занимает камеру — детектор с ним не стартует
        subprocess.call(["killall", "face_recognizer"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.Popen(cmd, shell=True, stdout=open("/tmp/yolo_detector.log", "ab"), stderr=subprocess.STDOUT)
        log("детектор запущен: %s" % cmd)
        time.sleep(3.0)
    except OSError as e:
        log("детектор не запустился: %s" % (e,))


def check_plan(plan):
    """Проверка плана до взлёта. Возвращает текст ошибки или None."""
    try:
        w = plan["window"]
        a, h = float(w["forward"]), float(w["height"])
        b = float(w.get("left", 0.0))
    except (KeyError, TypeError, ValueError):
        return "в плане нет window.forward / window.height"
    if not 0.4 <= a <= 3.0:
        return "окно в %.2f м от площадки: жду от 0,4 до 3 м" % a
    if abs(b) > 0.4:
        return "окно на %.2f м в стороне от оси ровера: больше 0,4 м" % b
    if not 0.5 <= h <= 2.0:
        return "высота центра окна %.2f м: жду от 0,5 до 2 м" % h
    for p in plan.get("search", {}).get("points", []) or []:
        if len(p) != 2 or float(p[0]) <= a:
            return "точка поиска %r не за окном" % (p,)
    return None


def make_drone(args, clock):
    if args.fake:
        from fake_offboard import FakeOffboard, Frame, World
        return FakeOffboard(clock, World(), drift_scale=0.05), Frame
    from simple_offboard_py import Frame, SimpleOffboard
    return SimpleOffboard(), Frame


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--config", default=os.path.join(here, "config.json"))
    ap.add_argument("--plan", help="план из файла JSON: полёт без ровера")
    ap.add_argument("--yes", action="store_true", help="с --plan: не спрашивать подтверждение взлёта")
    ap.add_argument("--rover", help="адрес ровера (иначе — по UDP-маяку)")
    ap.add_argument("--fake", action="store_true", help="имитатор вместо микродрона (на компьютере)")
    ap.add_argument("--speed", type=float, default=1.0, help="с --fake: ускорение времени")
    ap.add_argument("--once", action="store_true", help="один полёт и выход")
    a = ap.parse_args(argv)

    cfg = load_config(a.config)
    clock = Clock(a.speed if a.fake else 1.0)
    if a.fake:
        cfg.update({"hfov_deg": 100.0, "selfcheck_cmd": "", "start_detector": False})
    drone, frame = make_drone(a, clock)
    stop = {"flag": False}

    def on_signal(_sig, _frm):
        stop["flag"] = True
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, on_signal)

    if a.plan:
        link = NullLink(log)
    else:
        link = Link(clock, a.rover or cfg.get("rover_host") or None, int(cfg.get("rover_port", 8770)),
                    int(cfg.get("beacon_port", 8771)), log)
        link.start()

    log("программа микродрона %s; %s" % (VERSION, "имитатор" if a.fake else "simple_offboard_py"))
    try:
        while not stop["flag"]:
            ensure_detector(cfg)
            sc = selfcheck(cfg.get("selfcheck_cmd", "drone selfcheck"))
            log("предполётная проверка: %s" % ({True: "пройдена", False: "НЕ ПРОЙДЕНА", None: "не выполнена"}[sc["ok"]]))
            hello = {"t": "hello", "name": "whoop", "version": VERSION, "selfcheck": sc,
                     "rtsp_path": cfg.get("rtsp_path", "/h264"), "fake": bool(a.fake)}
            if a.plan:
                with open(a.plan, "r") as f:
                    plan = json.load(f)
                err = check_plan(plan)
                if err:
                    log("план не годится: %s" % err)
                    return 2
                if sc["ok"] is False:
                    log("взлёт запрещён: предполётная проверка не пройдена\n%s" % sc.get("raw", ""))
                    return 3
                if not a.yes:
                    input("Пульт в руках, SB готов к перехвату. Enter — взлёт, Ctrl+C — отмена: ")
            else:
                plan = wait_for_go(link, clock, hello, sc)
                if plan is None:
                    continue
            m = Mission(drone, frame, cfg, link, clock, log)
            res = m.run(plan)
            log("миссия: %s %s" % ("выполнена" if res["ok"] else "прервана", res["reason"]))
            if a.plan or a.once:
                return 0 if res["ok"] else 1
    except KeyboardInterrupt:
        log("остановлено")
        try:
            drone.land()
        except Exception as e:                          # noqa: BLE001
            log("посадка: %r" % (e,))
    return 0


def wait_for_go(link, clock, hello, sc):
    """Ждать от ровера plan, затем go. Возвращает план или None (начать заново)."""
    link.send(hello, keep=False)
    plan, plan_t, hello_t = None, 0.0, clock.time()
    while True:
        for m in link.poll():
            t = m.get("t")
            if t == "_connected":
                link.send(hello, keep=False)
            elif t == "plan":
                err = check_plan(m)
                if err is None and sc["ok"] is False:
                    err = "предполётная проверка не пройдена"
                if err:
                    log("план отклонён: %s" % err)
                    link.send({"t": "plan_bad", "reason": err})
                    plan = None
                else:
                    plan, plan_t = m, clock.time()
                    log("план принят: окно в %.2f м, высота %.2f м" % (float(m["window"]["forward"]), float(m["window"]["height"])))
                    link.send({"t": "plan_ok"})
            elif t == "go":
                if plan is None:
                    link.send({"t": "plan_bad", "reason": "go без плана"})
                elif link.age() > 2.0:
                    link.send({"t": "plan_bad", "reason": "связь неустойчива"})
                else:
                    return plan
            elif t == "abort":
                plan = None
        if plan is not None and clock.time() - plan_t > 60.0:
            log("план устарел: go не пришёл за 60 с")
            link.send({"t": "plan_bad", "reason": "план устарел"})
            plan = None
        if clock.time() - hello_t > 30.0 and plan is None:
            return None                                  # повторить проверку и hello
        clock.sleep(0.05)


if __name__ == "__main__":
    sys.exit(main())
