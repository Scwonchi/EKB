"""Миссия микродрона: взлёт с площадки ровера, пролёт окна, поиск людей, посадка.

Работает через интерфейс simple_offboard_py.SimpleOffboard (takeoff, land, set_position,
get_position, get_detections) и не зависит от сети и железа: связь и часы передаются снаружи,
поэтому ту же миссию гоняет имитатор на компьютере (fake_offboard.py, tests/).

Ход миссии
1. Взлёт над площадкой, подъём на высоту центра окна.
2. Прицеливание: детектор видит окно, микродрон измеряет, насколько центр окна в стороне и
   выше или ниже линии полёта.
3. Пролёт окна медленно, по прямой; пока окно в кадре, линия подправляется по камере. Так пролёт
   не зависит от ухода оценки положения (оптический поток над слабой текстурой).
4. За окном — зависание в двух (или больше) точках, в каждой поворот с остановками. Каждый
   найденный человек — пеленг; положение — пересечение пеленгов из разных точек: дистанцию по
   размеру рамки брать нельзя, пострадавший может быть закрыт до половины.
5. Тревоги и итоговые координаты — роверу; посадка на месте.

Отказы: команда abort, потеря связи до окна, выход времени полёта, нет данных о положении,
точка не достигнута — зависание и посадка на месте. За окном потеря связи миссию не прерывает:
сообщения копятся и уходят, когда связь вернётся.
"""
import math

from geometry import Camera, intersect, locate, ned_to_rover, range_from_height, rover_to_ned, wrap

DEFAULTS = {
    # --- камера и детектор (ИЗМЕРИТЬ: cam_check.py) ---
    "frame_w": 640,               # размер кадра, в пикселях которого детектор отдаёт рамки
    "frame_h": 640,
    "hfov_deg": 90.0,             # угол обзора курсовой камеры по горизонтали
    "cam_pitch_deg": 0.0,         # наклон камеры вверх
    "person_label": "person",
    "window_label": "window",     # "" — окна в модели нет: пролёт только по координатам
    "person_score": 0.5,
    "window_score": 0.4,
    # --- площадка и взлёт ---
    "pad_height_m": 0.25,         # высота площадки ровера над полом (ИЗМЕРИТЬ)
    "takeoff_height_m": 0.5,      # первый подъём над площадкой
    "z_jump_min_m": 0.08,         # скачок оценки высоты при сходе с площадки: больше этого — учесть
    "z_watch_m": 0.7,             # следить за скачком на этом пути от площадки
    # --- полёт ---
    "cruise_speed": 0.4,          # м/с
    "window_speed": 0.25,         # м/с сквозь окно
    "pos_tol_m": 0.12,
    "yaw_tol_deg": 8.0,
    "max_flight_s": 170.0,        # аккумулятора хватает примерно на 5 минут
    "link_loss_s": 2.0,           # до окна: связи с ровером нет дольше — посадка
    # --- окно ---
    "align_s": 1.5,               # столько смотреть на окно перед пролётом
    "align_min_hits": 4,
    "visual_min_dist_m": 0.35,    # ближе к окну камера его уже не видит целиком — линия не меняется
    "max_line_offset_m": 0.45,    # больше этого линию по камере не сдвигать
    "window_margin_m": 0.12,      # запас от края проёма до оси микродрона
    "fly_blind_if_no_window": True,   # окно не распознано: лететь по координатам от ровера
    "beyond_window_m": 1.0,       # первая точка поиска — на столько за окном
    # --- поиск ---
    "scan_height_m": 1.0,
    "scan_sector_deg": 240.0,     # сектор обзора, симметрично вперёд
    "scan_stops": 7,
    "scan_settle_s": 0.4,
    "scan_dwell_s": 1.5,
    "scan_min_hits": 4,
    "cluster_deg": 6.0,
    "merge_deg": 8.0,
    "baseline_m": 1.2,            # без точек в плане: обзор из трёх точек — за окном и на столько влево и вправо
    "person_height_m": 1.7,
    "max_victims": 3,
}


class Abort(Exception):
    """Миссию прервать: зависнуть и сесть на месте."""


class NullLink(object):
    """Полёт без ровера (проверка): сообщения печатаются."""
    required = False

    def __init__(self, log=None):
        self.log = log or (lambda s: None)
        self.sent = []

    def poll(self):
        return []

    def send(self, msg, keep=True):
        self.sent.append(msg)

    def age(self):
        return 0.0


class Mission(object):
    def __init__(self, drone, frame, cfg=None, link=None, clock=None, log=None):
        self.d, self.F = drone, frame
        self.cfg = dict(DEFAULTS)
        self.cfg.update(cfg or {})
        self.log = log or (lambda s: None)
        self.link = link or NullLink(self.log)
        self.clock = clock
        c = self.cfg
        self.cam = Camera(c["frame_w"], c["frame_h"], c["hfov_deg"], c["cam_pitch_deg"])
        self.phase = "idle"
        self.psi0 = 0.0
        self.home = (0.0, 0.0)
        self.z0 = 0.0                 # оценка z на площадке перед взлётом
        self.z_bias = 0.0             # поправка на скачок оценки высоты
        self.z_watch = False
        self.last = None              # последнее (x, y, z, yaw)
        self.t_takeoff = None
        self._state_t = -1e9
        self.before_window = True
        self.bbox_max = [0.0, 0.0]    # наибольшие координаты рамок — проверка frame_w / frame_h
        self.window_seen = False

    # ---------------------------------------------------------------- служебное
    def event(self, name, **kw):
        m = {"t": "event", "name": name}
        m.update(kw)
        self.log("%s %s" % (name, kw if kw else ""))
        self.link.send(m)

    def set_phase(self, phase):
        self.phase = phase
        self.event("phase", phase=phase)

    def pos(self):
        p = self.d.get_position()
        x, y, z, yaw = float(p[0]), float(p[1]), float(p[2]), float(p[5])
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            raise Abort("нет данных о положении")
        self.last = (x, y, z, yaw)
        return self.last

    def z_for(self, h):
        """Высота над полом -> z в LOCAL_NED (начало — на площадке, ось вниз)."""
        return self.z0 - (h - self.cfg["pad_height_m"]) + self.z_bias

    def height(self, z):
        return self.cfg["pad_height_m"] - (z - self.z0 - self.z_bias)

    def rover_xy(self, n, e):
        return ned_to_rover(n, e, self.psi0, self.home)

    def tick(self):
        """Раз за проход цикла: команды ровера, состояние роверу, сторожа."""
        now = self.clock.time()
        for m in self.link.poll():
            if m.get("t") == "abort":
                raise Abort("команда abort от ровера")
        if now - self._state_t >= 0.1 and self.last is not None:
            self._state_t = now
            x, y, z, yaw = self.last
            f, l = self.rover_xy(x, y)
            self.link.send({"t": "state", "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
                            "yaw": round(yaw, 3), "f": round(f, 3), "l": round(l, 3),
                            "h": round(self.height(z), 3), "phase": self.phase}, keep=False)
        if self.t_takeoff is not None and now - self.t_takeoff > self.cfg["max_flight_s"]:
            raise Abort("время полёта вышло")
        if self.before_window and getattr(self.link, "required", False) \
                and self.link.age() > self.cfg["link_loss_s"]:
            raise Abort("связь с ровером потеряна до пролёта окна")

    def detections(self, label, min_score):
        out = []
        for det in self.d.get_detections():
            self.bbox_max[0] = max(self.bbox_max[0], float(det.x2))
            self.bbox_max[1] = max(self.bbox_max[1], float(det.y2))
            if det.label == label and det.score >= min_score:
                out.append(self.cam.box(det))
        return out

    # ---------------------------------------------------------------- движение
    def goto(self, n, e, h, yaw, speed=None, tol=None, time_limit=None, on_tick=None):
        """Лететь в точку (N, E, высота над полом) с курсом yaw со скоростью не выше speed.
        Уставка едет к цели по прямой («морковка»), поэтому скорость задаём мы, а не контроллер.
        on_tick(пройденная доля) может вернуть новую цель (n, e, h) — так линия подправляется
        по камере. Возвращает True, если точка достигнута."""
        c = self.cfg
        speed = speed or c["cruise_speed"]
        tol = tol or c["pos_tol_m"]
        x0, y0, z0, _ = self.pos()
        h0 = self.height(z0)
        dist = math.sqrt((n - x0) ** 2 + (e - y0) ** 2 + (h - h0) ** 2)
        limit = time_limit or (dist / speed * 2.0 + 8.0)
        t0 = self.clock.time()
        ok_since = None
        while True:
            t = self.clock.time() - t0
            if t > limit:
                return False
            frac = 1.0 if dist < 1e-3 else min(1.0, (speed * t + 0.12) / dist)
            if on_tick is not None:
                new = on_tick(frac)
                if new is not None:
                    n, e, h = new
            sn, se, sh = x0 + (n - x0) * frac, y0 + (e - y0) * frac, h0 + (h - h0) * frac
            self.d.set_position(sn, se, self.z_for(sh), self.F.LOCAL_NED, yaw=yaw)
            self.clock.sleep(0.1)
            x, y, z, cur_yaw = self.pos()
            self._watch_z(z, self.z_for(sh), x, y)
            self.tick()
            if frac >= 1.0:
                err = math.sqrt((n - x) ** 2 + (e - y) ** 2 + (h - self.height(z)) ** 2)
                if err < tol and abs(wrap(yaw - cur_yaw)) < math.radians(c["yaw_tol_deg"]):
                    ok_since = ok_since if ok_since is not None else self.clock.time()
                    if self.clock.time() - ok_since >= 0.3:
                        return True
                else:
                    ok_since = None

    def hold(self, seconds, on_tick=None):
        """Висеть на месте."""
        x, y, z, yaw = self.pos()
        t0 = self.clock.time()
        while self.clock.time() - t0 < seconds:
            self.d.set_position(x, y, z, self.F.LOCAL_NED, yaw=yaw)
            self.clock.sleep(0.1)
            self.pos()
            self.tick()
            if on_tick is not None:
                on_tick()

    def _watch_z(self, z, z_cmd, x, y):
        """Скачок оценки высоты при сходе с площадки. Дальномер смотрел на площадку, а стал — на
        пол; если оценка высоты привязана к поверхности под дроном, она прыгает на высоту
        площадки, и контроллер увёл бы дрон вниз. Сдвигаем уставку вслед за оценкой: настоящая
        высота не меняется."""
        if not self.z_watch:
            return
        if math.hypot(x - self.home[0], y - self.home[1]) > self.cfg["z_watch_m"]:
            self.z_watch = False
            return
        dev = z - z_cmd
        if abs(dev) > self.cfg["z_jump_min_m"]:
            self.z_bias += dev
            self.event("z_jump", jump_m=round(dev, 3), bias_m=round(self.z_bias, 3))

    # ---------------------------------------------------------------- окно
    def _measure_window(self, seconds, dist):
        """Среднее положение центра окна относительно носа: (вправо, вверх) в метрах на
        расстоянии dist, число кадров с окном, оценка расстояния по ширине окна."""
        c = self.cfg
        acc = []

        def look():
            best = None
            for b in self.detections(c["window_label"], c["window_score"]):
                if best is None or b["score"] > best["score"]:
                    best = b
            if best is not None:
                acc.append(best)

        self.hold(seconds, look)
        if len(acc) < c["align_min_hits"]:
            return None
        az = sum(b["az"] for b in acc) / len(acc)
        el = sum(b["el"] for b in acc) / len(acc)
        whole = [b for b in acc if not b["cut"]]
        aw = sum(b["aw"] for b in whole) / len(whole) if whole else None
        return {"right": dist * math.tan(az), "up": dist * math.tan(el), "hits": len(acc), "aw": aw}

    def fly_through_window(self, win):
        """Пролёт окна. win: forward, left, height, width, height_size (метры, оси ровера от площадки)."""
        c = self.cfg
        a, b, hw = float(win["forward"]), float(win.get("left", 0.0)), float(win["height"])
        width, ht = float(win.get("width", 0.85)), float(win.get("height_size", 0.70))
        frame = float(win.get("frame", 0.05))
        # 1. Над площадкой — на высоту центра окна
        self.set_phase("climb")
        if not self.goto(self.home[0], self.home[1], hw, self.psi0, speed=0.3):
            raise Abort("не вышел на высоту окна")
        # 2. Прицеливание по камере
        off_l, off_u = b, 0.0                       # линия полёта: влево от оси и выше плана
        use_cam = bool(c["window_label"])
        if use_cam:
            self.set_phase("align")
            m = self._measure_window(c["align_s"], a)
            if m is None:
                self.event("window_not_seen")
                if not c["fly_blind_if_no_window"]:
                    raise Abort("окно не распознано")
                use_cam = False
            else:
                self.window_seen = True
                off_l, off_u = -m["right"], m["up"]
                msg = {"left_m": round(off_l, 3), "up_m": round(off_u, 3), "hits": m["hits"]}
                if m["aw"]:
                    d_est = width / (2.0 * math.tan(m["aw"] / 2.0))
                    msg["dist_by_width_m"] = round(d_est, 2)
                    if abs(d_est - a) > 0.35 * a:
                        self.event("camera_check", note="расстояние до окна по камере %.2f м, по роверу %.2f м: "
                                                        "проверьте hfov_deg и frame_w" % (d_est, a))
                self.event("window_aligned", **msg)
        lim = c["max_line_offset_m"]
        if abs(off_l - b) > lim or abs(off_u) > lim:
            raise Abort("окно слишком далеко от оси: %.2f м вбок, %.2f м по высоте" % (off_l - b, off_u))
        # 3. Пролёт: прямая от площадки сквозь окно, линия подправляется по камере
        self.set_phase("window")
        self.z_watch = True
        through = a + c["beyond_window_m"]
        state = {"l": off_l, "u": off_u, "last_err": None}

        def target():
            n, e = rover_to_ned(through, state["l"], self.psi0, self.home)
            return n, e, hw + state["u"]

        def correct(_frac):
            x, y, z, _ = self.last
            f, l = self.rover_xy(x, y)
            d = a - f
            if not use_cam or d < c["visual_min_dist_m"]:
                return None
            best = None
            for bx in self.detections(c["window_label"], c["window_score"]):
                if best is None or bx["score"] > best["score"]:
                    best = bx
            if best is None:
                return None
            # где центр окна относительно дрона сейчас -> где должна идти линия
            l_goal = l - d * math.tan(best["az"])
            u_goal = (self.height(z) - hw) + d * math.tan(best["el"])
            state["last_err"] = (l_goal - l, u_goal - (self.height(z) - hw))
            k = 0.3
            state["l"] = max(b - lim, min(b + lim, state["l"] + k * (l_goal - state["l"])))
            state["u"] = max(-lim, min(lim, state["u"] + k * (u_goal - state["u"])))
            return target()

        # 3а. До точки перед окном: линия ещё подстраивается
        pre = max(0.0, a - c["visual_min_dist_m"])
        n, e = rover_to_ned(pre, state["l"], self.psi0, self.home)

        def correct_pre(frac):
            t = correct(frac)
            if t is None:
                return None
            pn, pe = rover_to_ned(pre, state["l"], self.psi0, self.home)
            return pn, pe, hw + state["u"]

        if pre > 0.05 and not self.goto(n, e, hw + state["u"], self.psi0, speed=c["window_speed"], on_tick=correct_pre):
            raise Abort("не вышел к окну")
        # 3б. Перед самым окном: проверить, что стоим на линии и линия — в проёме
        x, y, z, _ = self.pos()
        f, l = self.rover_xy(x, y)
        half_w = width / 2.0 - frame - c["window_margin_m"]
        half_h = ht / 2.0 - frame - c["window_margin_m"]
        if state["last_err"] is not None:
            el_, eu_ = state["last_err"]
            if abs(el_) > half_w or abs(eu_) > half_h:
                # последнее измерение говорит, что идём мимо проёма: довернуть и перепроверить
                self.event("window_recheck", left_err_m=round(el_, 3), up_err_m=round(eu_, 3))
                self.hold(1.0, lambda: correct(0))
                n, e = rover_to_ned(pre, state["l"], self.psi0, self.home)
                if not self.goto(n, e, hw + state["u"], self.psi0, speed=0.15):
                    raise Abort("не выровнялся перед окном")
                el_, eu_ = state["last_err"]
                if abs(el_) > half_w or abs(eu_) > half_h:
                    raise Abort("не попадаю в проём окна: %.2f м вбок, %.2f м по высоте" % (el_, eu_))
        # 3в. Сквозь окно по прямой, без поправок
        self.before_window = False
        n, e, h = target()
        if not self.goto(n, e, h, self.psi0, speed=c["window_speed"]):
            raise Abort("не прошёл окно")
        self.z_watch = False
        self.event("window_passed", line_left_m=round(state["l"], 3), line_up_m=round(state["u"], 3),
                   by_camera=bool(use_cam))
        return through, state["l"]

    # ---------------------------------------------------------------- поиск
    def scan_point(self, idx, n, e, h, alarms):
        """Обзор из точки: поворот с остановками, люди -> пеленги. Возвращает наблюдения."""
        c = self.cfg
        if not self.goto(n, e, h, self.last[3] if self.last else self.psi0):
            self.event("scan_point_missed", point=idx)       # сканируем оттуда, куда долетели
        obs = []
        stops = max(1, int(c["scan_stops"]))
        sector = math.radians(c["scan_sector_deg"])
        for k in range(stops):
            rel = 0.0 if stops == 1 else -sector / 2.0 + sector * k / (stops - 1)
            yaw = wrap(self.psi0 + rel)
            x, y, _, _ = self.pos()
            self.goto(x, y, h, yaw, speed=0.2, tol=0.2, time_limit=6.0)
            self.hold(c["scan_settle_s"])
            clusters = []

            def look():
                x_, y_, _, yaw_ = self.last
                for b in self.detections(c["person_label"], c["person_score"]):
                    brg = wrap(yaw_ + b["az"])
                    for cl in clusters:
                        if abs(wrap(brg - cl["bearing"])) < math.radians(c["cluster_deg"]):
                            cl["hits"] += 1
                            cl["bearing"] = wrap(cl["bearing"] + wrap(brg - cl["bearing"]) / cl["hits"])
                            cl["ah"] += (b["ah"] - cl["ah"]) / cl["hits"]
                            cl["score"] = max(cl["score"], b["score"])
                            cl["cut"] = cl["cut"] and b["cut"]
                            break
                    else:
                        clusters.append({"bearing": brg, "ah": b["ah"], "score": b["score"], "cut": b["cut"],
                                         "hits": 1, "n": x_, "e": y_})

            self.hold(c["scan_dwell_s"], look)
            for cl in clusters:
                if cl["hits"] < c["scan_min_hits"]:
                    continue
                dup = None
                for o in obs:
                    if abs(wrap(o["bearing"] - cl["bearing"])) < math.radians(c["merge_deg"]):
                        dup = o
                        break
                if dup is None:
                    cl["point"] = idx
                    obs.append(cl)
                    self._alarm(cl, alarms)
                elif (dup["cut"] and not cl["cut"]) or (dup["cut"] == cl["cut"] and cl["hits"] > dup["hits"]):
                    dup.update({k2: cl[k2] for k2 in ("bearing", "ah", "score", "cut", "hits", "n", "e")})
        self.event("scan_done", point=idx, seen=len(obs))
        return obs

    def _alarm(self, o, alarms):
        """Тревога при обнаружении. Первый раз человека видно из одной точки: направление и
        грубая дистанция (по высоте рамки). Если тот же человек уже замечен из другой точки
        (пеленги пересекаются, видимый размер сходится), новой тревоги нет — уточняется старая."""
        c = self.cfg
        same = None
        for al in alarms:
            for prev in al["obs"]:
                if prev["point"] == o["point"]:
                    continue
                x = intersect((prev["n"], prev["e"]), prev["bearing"], (o["n"], o["e"]), o["bearing"])
                if x is None:
                    continue
                (n, e), r1, r2 = x
                ratio = (prev["ah"] * r1) / max(o["ah"] * r2, 1e-6)
                if 0.3 <= r1 <= 7.0 and 0.3 <= r2 <= 7.0 and 1 / 1.7 < ratio < 1.7:
                    same = (al, n, e, r2)
                    break
            if same:
                break
        if same:
            al, n, e, r = same
            al["obs"].append(o)
            approx, update = False, True
        else:
            r = range_from_height(o["ah"], c["person_height_m"])
            n, e = o["n"] + r * math.cos(o["bearing"]), o["e"] + r * math.sin(o["bearing"])
            al = {"id": len(alarms) + 1, "obs": [o]}
            alarms.append(al)
            approx, update = True, False
        f, l = self.rover_xy(n, e)
        df, dl = self.rover_xy(o["n"], o["e"])
        self.link.send({"t": "detection", "id": al["id"], "update": update, "point": o["point"],
                        "bearing_deg": round(math.degrees(wrap(o["bearing"] - self.psi0)), 1),
                        "range_m": round(r, 2), "range_approx": approx, "score": round(o["score"], 2),
                        "f": round(f, 2), "l": round(l, 2), "drone_f": round(df, 2), "drone_l": round(dl, 2)})
        self.log("%s: человек №%d, пеленг %+.0f° от оси ровера, %s%.1f м"
                 % ("уточнение" if update else "ТРЕВОГА", al["id"],
                    math.degrees(wrap(o["bearing"] - self.psi0)), "примерно " if approx else "", r))

    def search(self, plan, first_f, first_l):
        c = self.cfg
        s = plan.get("search", {})
        h = float(s.get("height", c["scan_height_m"]))
        pts = s.get("points")
        if not pts:
            pts = [[first_f, first_l], [first_f, first_l + c["baseline_m"]], [first_f, first_l - c["baseline_m"]]]
        bounds = s.get("bounds")                       # [f_min, f_max, l_min, l_max] в осях ровера
        inside = None
        if bounds:
            def inside(n, e):
                f, l = self.rover_xy(n, e)
                return bounds[0] <= f <= bounds[1] and bounds[2] <= l <= bounds[3]
        self.set_phase("search")
        alarms, per_point = [], []
        for i, (f, l) in enumerate(pts):
            n, e = rover_to_ned(float(f), float(l), self.psi0, self.home)
            per_point.append(self.scan_point(i, n, e, h, alarms))
        # Пересечение пеленгов из разных точек
        found, rest = locate(per_point, inside=inside)
        victims = [{"n": v["n"], "e": v["e"], "approx": False, "score": v["score"], "rays": v["rays"]}
                   for v in found]
        # Кого видели только из одной точки — по пеленгу и грубой дистанции
        for o in rest:
            r = range_from_height(o["ah"], c["person_height_m"])
            n, e = o["n"] + r * math.cos(o["bearing"]), o["e"] + r * math.sin(o["bearing"])
            # тот же человек, что уже найден: пеленг проходит рядом с ним
            if any(abs(wrap(math.atan2(v["e"] - o["e"], v["n"] - o["n"]) - o["bearing"])) < math.radians(10.0)
                   or math.hypot(v["n"] - n, v["e"] - e) < 0.8 for v in victims):
                continue
            victims.append({"n": n, "e": e, "approx": True, "score": o["score"], "rays": 1})
        victims.sort(key=lambda v: (v["approx"], -v["score"]))
        victims = victims[:int(c["max_victims"])]
        out = []
        for k, v in enumerate(victims):
            f, l = self.rover_xy(v["n"], v["e"])
            out.append({"id": k + 1, "f": round(f, 2), "l": round(l, 2), "approx": v["approx"],
                        "score": round(v["score"], 2), "rays": v["rays"]})
        self.link.send({"t": "victims", "count": len(out), "victims": out})
        self.log("Найдено людей: %d %s" % (len(out), out))
        return out

    # ---------------------------------------------------------------- миссия целиком
    def run(self, plan):
        """plan: {"window": {...}, "search": {...}} — оси ровера от площадки, метры.
        Возвращает {"ok", "reason", "victims", "window_seen"}."""
        c = self.cfg
        result = {"ok": False, "reason": "", "victims": [], "window_seen": False}
        flying = False
        try:
            x, y, z, yaw = self.pos()
            self.home, self.z0, self.psi0 = (x, y), z, yaw
            self.z_bias, self.before_window = 0.0, True
            self.set_phase("takeoff")
            self.t_takeoff = self.clock.time()
            flying = True
            if not self.d.takeoff(c["takeoff_height_m"]):
                raise Abort("взлёт не удался")
            self.pos()
            self.tick()
            through, line_l = self.fly_through_window(plan["window"])
            result["window_seen"] = self.window_seen
            result["victims"] = self.search(plan, through, line_l)
            result["ok"] = True
        except Abort as a:
            result["reason"] = str(a)
            self.event("abort", reason=str(a))
        except Exception as ex:                      # любая ошибка в полёте — садимся
            result["reason"] = "ошибка программы: %r" % (ex,)
            self.event("abort", reason=result["reason"])
        if self.bbox_max[0] > c["frame_w"] + 2 or self.bbox_max[1] > c["frame_h"] + 2:
            self.event("camera_check", note="рамки детектора выходят за кадр %dx%d (до %.0f, %.0f): "
                                            "исправьте frame_w, frame_h"
                                            % (c["frame_w"], c["frame_h"], self.bbox_max[0], self.bbox_max[1]))
        if flying:
            self.phase = "land"
            self.link.send({"t": "event", "name": "phase", "phase": "land"})
            try:
                self.d.land()
            except Exception as ex:
                self.log("посадка: %r" % (ex,))
        self.phase = "done"
        self.t_takeoff = None
        self.link.send({"t": "done", "ok": result["ok"], "reason": result["reason"],
                        "victims": result["victims"]})
        return result
