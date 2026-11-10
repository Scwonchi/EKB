"""Проверки программы микродрона на компьютере, без микродрона и без pytest:

    python3 tests/test_whoop.py          # или: python3 -m pytest tests

Миссия идёт на имитаторе (fake_offboard.py) в виртуальном времени.
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from clock import SimClock                                              # noqa: E402
from fake_offboard import Detection, FakeOffboard, Frame, World         # noqa: E402
from geometry import (Camera, intersect, least_squares_point, locate, ned_to_rover,  # noqa: E402
                      rover_to_ned, wrap)
from mission import Mission, NullLink                                   # noqa: E402
from whoop_agent import check_plan                                      # noqa: E402

PERSONS = [(3.0, 0.6, 1.7, 0.0), (2.6, -1.3, 1.7, 0.5), (4.0, 1.5, 1.8, 0.0)]


def fly(world=None, cfg=None, plan=None, link=None, **kw):
    clock = SimClock()
    w = world or World(persons=list(PERSONS))
    drone = FakeOffboard(clock, w, **kw)
    base = {"hfov_deg": 100.0, "pad_height_m": w.pad_h}
    base.update(cfg or {})
    m = Mission(drone, Frame, base, link, clock)
    plan = plan or {"window": {"forward": w.window_n, "left": -w.window_e, "height": w.window_h}}
    return m.run(plan), drone, m, clock, w


def errors(result, world):
    """Для каждого человека сцены — ошибка ближайшей найденной точки по осям, м."""
    out = []
    for p in world.persons:
        best = min(result["victims"], key=lambda v: math.hypot(v["f"] - p[0], v["l"] + p[1]))
        out.append(max(abs(best["f"] - p[0]), abs(best["l"] + p[1])))
    return out


# ---------------------------------------------------------------- геометрия

def test_axes_roundtrip():
    for psi in (0.0, 0.7, -2.5):
        n, e = rover_to_ned(1.2, -0.4, psi, (0.3, -0.2))
        f, l = ned_to_rover(n, e, psi, (0.3, -0.2))
        assert abs(f - 1.2) < 1e-9 and abs(l + 0.4) < 1e-9
    assert rover_to_ned(1, 0, 0) == (1.0, 0.0)             # вперёд = N при курсе 0
    n, e = rover_to_ned(0, 1, 0)                           # влево = минус E
    assert abs(n) < 1e-9 and abs(e + 1) < 1e-9


def test_camera_angles_and_fov():
    cam = Camera(640, 480, 90.0)
    az, el = cam.angles(640, 240)                          # правый край кадра — половина угла обзора
    assert abs(math.degrees(az) - 45) < 1e-6 and abs(el) < 1e-9
    assert cam.angles(320, 0)[1] > 0                       # верх кадра — вверх
    # предмет 0,85 м на 1,5 м занимает w px -> обратно получаем 90°
    w_px = 0.85 / 1.5 * cam.f
    assert abs(cam.hfov_from_object(w_px, 0.85, 1.5) - 90.0) < 0.01


def test_intersect_and_least_squares():
    x = intersect((0, 0), math.radians(45), (0, 2), math.radians(-45))
    assert abs(x[0][0] - 1) < 1e-9 and abs(x[0][1] - 1) < 1e-9
    assert intersect((0, 0), 0.0, (0, 1), 0.02) is None                # почти параллельны
    assert intersect((0, 0), math.radians(45), (0, 2), math.radians(135)) is None   # позади
    p = least_squares_point([((0, 0), math.radians(45)), ((0, 2), math.radians(-45)), ((2, 1), math.pi)])
    assert abs(p[0] - 1) < 1e-6 and abs(p[1] - 1) < 1e-6


def test_locate_three_people_with_noise():
    rng = random.Random(1)
    people = [(3.0, 0.5), (2.5, -1.4), (4.2, 1.6)]
    pts = [(1.8, 0.0), (1.8, -1.2), (1.8, 1.2)]
    per_point = []
    for (pn, pe) in pts:
        obs = []
        for (n, e) in people:
            r = math.hypot(n - pn, e - pe)
            obs.append({"n": pn, "e": pe, "bearing": math.atan2(e - pe, n - pn) + rng.gauss(0, math.radians(1.0)),
                        "ah": 2 * math.atan(0.85 / r), "score": 0.8})
        rng.shuffle(obs)
        per_point.append(obs)
    found, rest = locate(per_point)
    assert len(found) == 3 and not rest
    for (n, e) in people:
        assert min(math.hypot(v["n"] - n, v["e"] - e) for v in found) < 0.25


# ---------------------------------------------------------------- миссия

def test_mission_finds_three_people():
    r, drone, m, clock, w = fly()
    assert r["ok"], r["reason"]
    assert drone.crashed is None and not drone.flying
    assert len(r["victims"]) == 3
    assert max(errors(r, w)) < 0.5                         # допуск кейса: 0,5 м по каждой оси
    assert clock.time() < 150                              # в пределах аккумулятора (~5 минут)


def test_window_off_axis_passed_by_camera():
    """Окно на 20 см в стороне и на 10 см выше, чем сообщил ровер: камера выводит в проём."""
    w = World(window_e=0.20, window_h=1.30, persons=[(3.0, 0.5, 1.7, 0.0)])
    r, drone, m, _, _ = fly(w, plan={"window": {"forward": 0.8, "left": 0.0, "height": 1.2}})
    assert r["ok"], r["reason"]
    assert drone.crashed is None and r["window_seen"]
    de, dh = drone.log[0][1], drone.log[0][2]
    assert abs(de) < 0.08 and abs(dh) < 0.08               # прошёл почти через центр


def test_same_offset_without_camera_hits_frame():
    """То же окно, но модель без класса «окно»: по одним координатам — удар о раму.
    Проверка, что имитатор действительно наказывает за пролёт мимо проёма."""
    w = World(window_e=0.32, persons=[(3.0, 0.5, 1.7, 0.0)])
    r, drone, _, _, _ = fly(w, cfg={"window_label": ""}, plan={"window": {"forward": 0.8, "left": 0.0, "height": 1.2}})
    assert drone.crashed is not None


def test_drift_does_not_break_window_pass():
    """Оценка положения уходит на 10% пути (слабая текстура пола): окно всё равно пройдено."""
    for seed in range(6):
        w = World(window_e=0.12, persons=[(3.2, -0.4, 1.7, 0.0)])
        r, drone, _, _, _ = fly(w, drift_scale=0.10, seed=seed,
                                plan={"window": {"forward": 0.8, "left": 0.0, "height": 1.2}})
        assert r["ok"] and drone.crashed is None, (seed, r["reason"], drone.crashed)


def test_height_jump_when_leaving_pad():
    """Оценка высоты привязана к поверхности под дроном: при сходе с площадки она прыгает на
    высоту площадки. Программа сдвигает уставку, и окно пройдено на своей высоте — даже без камеры."""
    w = World(persons=[(3.0, 0.5, 1.7, 0.0)])
    r, drone, m, _, _ = fly(w, cfg={"window_label": ""}, z_mode="ground")
    assert r["ok"] and drone.crashed is None, (r["reason"], drone.crashed)
    assert abs(m.z_bias + w.pad_h) < 0.08
    assert abs(drone.log[0][2]) < 0.10                     # высота в плоскости окна


def test_half_hidden_person_located():
    """Человек закрыт предметом до половины: дистанция по высоте рамки врала бы вдвое,
    пересечение пеленгов даёт верное место."""
    w = World(persons=[(3.4, 0.3, 1.7, 0.5)])
    r, _, _, _, _ = fly(w)
    assert r["ok"] and len(r["victims"]) == 1 and not r["victims"][0]["approx"]
    assert errors(r, w)[0] < 0.3


def test_one_alarm_per_person():
    """Человека видно из трёх точек: тревога одна, остальные сообщения — уточнения."""
    link = NullLink()
    w = World(persons=[(3.0, 0.5, 1.7, 0.0)])
    r, _, _, _, _ = fly(w, link=link)
    det = [m for m in link.sent if m["t"] == "detection"]
    assert len([m for m in det if not m["update"]]) == 1
    assert len(det) >= 2 and all(m["id"] == 1 for m in det)
    assert [m["t"] for m in link.sent][-1] == "done"


def test_no_people_no_alarms():
    link = NullLink()
    r, _, _, _, _ = fly(World(persons=[]), link=link)
    assert r["ok"] and r["victims"] == []
    assert not [m for m in link.sent if m["t"] == "detection"]


# ---------------------------------------------------------------- отказы

class ScriptLink(NullLink):
    """Связь по сценарию: abort или обрыв в заданный момент."""
    required = True

    def __init__(self, clock_ref, abort_at=None, lost_at=None):
        NullLink.__init__(self)
        self.c, self.abort_at, self.lost_at = clock_ref, abort_at, lost_at

    def poll(self):
        if self.abort_at is not None and self.c[0].time() >= self.abort_at:
            self.abort_at = None
            return [{"t": "abort"}]
        return []

    def age(self):
        if self.lost_at is not None and self.c[0].time() >= self.lost_at:
            return self.c[0].time() - self.lost_at
        return 0.0


def _fly_with(link_factory, **kw):
    ref = [None]
    link = link_factory(ref)
    clock = SimClock()
    ref[0] = clock
    w = World(persons=[(3.0, 0.5, 1.7, 0.0)])
    drone = FakeOffboard(clock, w, **kw)
    m = Mission(drone, Frame, {"hfov_deg": 100.0, "pad_height_m": w.pad_h}, link, clock)
    r = m.run({"window": {"forward": 0.8, "left": 0.0, "height": 1.2}})
    return r, drone, link


def test_abort_command_lands():
    r, drone, _ = _fly_with(lambda ref: ScriptLink(ref, abort_at=4.0))
    assert not r["ok"] and "abort" in r["reason"]
    assert not drone.flying and drone.n < 0.8              # сел, окно не пройдено


def test_link_lost_before_window_lands():
    r, drone, _ = _fly_with(lambda ref: ScriptLink(ref, lost_at=3.0))
    assert not r["ok"] and "связь" in r["reason"]
    assert not drone.flying and drone.n < 0.8


def test_link_lost_after_window_mission_continues():
    """За окном связь пропала: миссия продолжается, итог копится для отправки."""
    r, drone, link = _fly_with(lambda ref: ScriptLink(ref, lost_at=25.0))
    assert r["ok"] and len(r["victims"]) == 1
    assert link.sent[-1]["t"] == "done"


def test_takeoff_failure():
    r, drone, _ = _fly_with(lambda ref: NullLink(), takeoff_ok=False)
    assert not r["ok"] and "взлёт" in r["reason"]


def test_flight_time_limit_lands():
    clock = SimClock()
    w = World()
    drone = FakeOffboard(clock, w)
    m = Mission(drone, Frame, {"hfov_deg": 100.0, "pad_height_m": w.pad_h, "max_flight_s": 30.0}, None, clock)
    r = m.run({"window": {"forward": 0.8, "left": 0.0, "height": 1.2}})
    assert not r["ok"] and "время" in r["reason"] and not drone.flying


def test_window_not_recognised_policy():
    w = World(persons=[(3.0, 0.5, 1.7, 0.0)])
    r, drone, _, _, _ = fly(w, cfg={"fly_blind_if_no_window": False}, labels=("person",))
    assert not r["ok"] and "окно не распознано" in r["reason"] and drone.n < 0.3
    r, drone, _, _, _ = fly(w, labels=("person",))         # по умолчанию — лететь по координатам ровера
    assert r["ok"] and not r["window_seen"] and drone.crashed is None


def test_wrong_frame_size_is_reported():
    """frame_w в настройках меньше настоящего кадра: программа об этом сообщает."""
    link = NullLink()
    fly(World(persons=[(3.0, 0.5, 1.7, 0.0)]), cfg={"frame_w": 320, "frame_h": 320}, link=link)
    assert [m for m in link.sent if m.get("name") == "camera_check"]


def test_plan_validation():
    ok = {"window": {"forward": 0.8, "height": 1.2}}
    assert check_plan(ok) is None
    assert check_plan({}) is not None
    assert check_plan({"window": {"forward": 5.0, "height": 1.2}}) is not None
    assert check_plan({"window": {"forward": 0.8, "left": 0.9, "height": 1.2}}) is not None
    assert check_plan({"window": {"forward": 0.8, "height": 1.2}, "search": {"points": [[0.5, 0]]}}) is not None


def test_detection_object_fields():
    d = Detection("person", 0.9, 10, 20, 110, 220)
    b = Camera(640, 640, 90).box(d)
    assert b["label"] == "person" and not b["cut"] and b["ah"] > b["aw"]
    assert Camera(640, 640, 90).box(Detection("person", 0.9, 0, 20, 110, 220))["cut"]
    assert abs(wrap(3 * math.pi)) - math.pi < 1e-9


if __name__ == "__main__":
    import traceback
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    bad = 0
    for name, fn in tests:
        try:
            import inspect
            if inspect.signature(fn).parameters:
                continue
            fn()
            print("ok   " + name)
        except Exception:                                  # noqa: BLE001
            bad += 1
            print("FAIL " + name)
            traceback.print_exc()
    print("итого: %d прошло, %d упало" % (len(tests) - bad, bad))
    sys.exit(1 if bad else 0)
