"""Имитатор микродрона с тем же интерфейсом, что simple_offboard_py.SimpleOffboard.

Нужен, чтобы проверять программу управления на компьютере, без микродрона: взлёт с площадки
ровера, пролёт окна, поиск людей. Моделирует то, что влияет на миссию:

* уход оценки положения (оптический поток над слабой текстурой): ошибка масштаба и дрейф;
* скачок оценки высоты, когда дальномер сходит с площадки ровера на пол (два варианта);
* курсовую камеру с детектором: рамки окна и людей в пикселях, шум, обрезка краем кадра,
  люди, наполовину закрытые предметами;
* столкновение с рамой окна, если микродрон идёт мимо проёма.

Это не модель динамики PX4: скорость и ускорение ограничены, и всё.
"""
import math
import random

from geometry import wrap


class Frame(object):
    BODY_NED = "BODY_NED"
    LOCAL_NED = "LOCAL_NED"


class Detection(object):
    def __init__(self, label, score, x1, y1, x2, y2, class_id=0):
        self.label, self.score, self.class_id = label, score, class_id
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2
        self.z1 = self.z2 = 0.0


class World(object):
    """Сцена в «настоящих» осях: N вперёд от площадки по оси ровера, E вправо, h — над полом."""

    def __init__(self, window_n=0.8, window_e=0.0, window_h=1.2, window_w=0.85, window_ht=0.70,
                 frame_w=0.05, persons=None, pad_h=0.25, pad_half=0.12):
        self.window_n, self.window_e, self.window_h = window_n, window_e, window_h
        self.window_w, self.window_ht, self.frame_w = window_w, window_ht, frame_w
        # человек: (N, E, рост, доля роста, закрытая снизу предметом)
        self.persons = persons if persons is not None else [(3.0, 0.5, 1.7, 0.0)]
        self.pad_h, self.pad_half = pad_h, pad_half


class FakeOffboard(object):
    def __init__(self, clock, world=None, seed=0, start_yaw=0.3, drift_scale=0.0, drift_rate=0.0,
                 z_mode="takeoff", frame_w=640, frame_h=640, hfov_deg=100.0, cam_pitch_deg=0.0,
                 max_speed=1.0, det_rate=0.9, takeoff_ok=True, labels=("person", "window")):
        """z_mode: "takeoff" — высота считается от точки взлёта, скачка нет;
        "ground" — от поверхности под дроном: при сходе с площадки оценка прыгает на её высоту."""
        self.clock, self.world, self.rng = clock, world or World(), random.Random(seed)
        self.yaw0 = start_yaw                              # курс в LOCAL_NED при взлёте («север» дрона)
        self.drift_scale, self.drift_rate, self.z_mode = drift_scale, drift_rate, z_mode
        self.fw, self.fh = frame_w, frame_h
        self.f_px = (frame_w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.cam_pitch = math.radians(cam_pitch_deg)
        self.max_speed, self.det_rate, self.takeoff_ok, self.labels = max_speed, det_rate, takeoff_ok, labels
        w = self.world
        # Настоящее состояние в осях сцены: n, e, h над полом, курс относительно оси ровера
        self.n, self.e, self.h, self.psi = 0.0, 0.0, w.pad_h, 0.0
        self.err_n = self.err_e = 0.0                      # накопленная ошибка оценки (оси сцены)
        self.armed = self.flying = False
        self.sp = None                                     # уставка: ("pos", n, e, h, psi) | ("vel", vn, ve, vh, psi_rate|None, psi)
        self.crashed = None
        self.t = clock.time()
        self.log = []

    # ---------- оценка, которую видит программа ----------
    def _ground(self):
        w = self.world
        return w.pad_h if abs(self.n) <= w.pad_half and abs(self.e) <= w.pad_half else 0.0

    def _est(self):
        """Оценка в LOCAL_NED: поворот на yaw0, ошибка положения, высота по выбранному режиму."""
        n, e = self.n + self.err_n, self.e + self.err_e
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        x, y = n * c - e * s, n * s + e * c
        ref = self._ground() if self.z_mode == "ground" else self.world.pad_h
        return x, y, -(self.h - ref), wrap(self.psi + self.yaw0)

    def _to_scene(self, x, y):
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        return x * c + y * s - self.err_n, -x * s + y * c - self.err_e

    def _h_from_z(self, z):
        ref = self._ground() if self.z_mode == "ground" else self.world.pad_h
        return ref - z

    # ---------- шаг модели ----------
    def _advance(self):
        now = self.clock.time()
        while self.t < now - 1e-9:
            dt = min(0.02, now - self.t)
            self.t += dt
            self._step(dt)

    def _step(self, dt):
        if not self.flying or self.sp is None or self.crashed:
            return
        w = self.world
        n0 = self.n
        if self.sp[0] == "pos":
            _, x, y, z, yaw = self.sp
            tn, te = self._to_scene(x, y)
            th = self._h_from_z(z)
            dn, de, dh = tn - self.n, te - self.e, th - self.h
            d = math.sqrt(dn * dn + de * de + dh * dh)
            v = min(self.max_speed, 2.0 * d)
            if d > 1e-6:
                self.n += dn / d * v * dt
                self.e += de / d * v * dt
                self.h += dh / d * v * dt
                moved = v * dt
            else:
                moved = 0.0
            if yaw is not None:
                self.psi += max(-1.5 * dt, min(1.5 * dt, wrap(yaw - self.yaw0 - self.psi)))
        else:
            _, vn, ve, vh, rate, yaw = self.sp
            self.n += vn * dt
            self.e += ve * dt
            self.h += vh * dt
            moved = math.sqrt(vn * vn + ve * ve) * dt
            if rate:
                self.psi += rate * dt
        self.psi = wrap(self.psi)
        self.h = max(self.h, self._ground())
        # Уход оценки: доля пройденного пути и медленный дрейф
        if moved > 0:
            ang = self.rng.uniform(0, 2 * math.pi)
            self.err_n += self.drift_scale * moved * math.cos(ang) * 0.5 + self.drift_scale * moved * 0.7
            self.err_e += self.drift_scale * moved * math.sin(ang) * 0.5 + self.drift_scale * moved * 0.5
        self.err_n += self.rng.gauss(0, self.drift_rate) * dt
        self.err_e += self.rng.gauss(0, self.drift_rate) * dt
        # Проход плоскости окна: внутри проёма (с запасом на размер дрона) или удар о раму
        if (n0 - w.window_n) * (self.n - w.window_n) < 0:
            half_w = w.window_w / 2 - w.frame_w - 0.08
            half_h = w.window_ht / 2 - w.frame_w - 0.06
            de, dh = self.e - w.window_e, self.h - w.window_h
            if abs(de) > half_w or abs(dh) > half_h:
                if abs(de) < w.window_w / 2 + 0.1 and abs(dh) < w.window_ht / 2 + 0.1:
                    self.crashed = "удар о раму окна: мимо центра на %.2f м вбок, %.2f м по высоте" % (de, dh)
                else:
                    self.crashed = "пролёт мимо окна: %.2f м вбок, %.2f м по высоте" % (de, dh)
            self.log.append(("window", round(de, 3), round(dh, 3)))

    # ---------- интерфейс SimpleOffboard ----------
    def takeoff(self, height=1.0):
        self._advance()
        if not self.takeoff_ok:
            self.clock.sleep(5.0)
            return False
        x, y, z, yaw = self._est()
        self.armed = self.flying = True
        self.sp = ("pos", x, y, z - height, yaw)
        for _ in range(100):
            self.clock.sleep(0.1)
            self._advance()
            if abs(self._est()[2] - (z - height)) < 0.05:
                return True
        return False

    def land(self):
        self._advance()
        while self.flying and self.h > self._ground() + 0.01 and not self.crashed:
            self.sp = ("vel", 0.0, 0.0, -0.4, None, None)
            self.clock.sleep(0.1)
            self._advance()
        self.flying = self.armed = False
        self.sp = None

    def navigate(self, x, y, z, frame=Frame.LOCAL_NED, yaw=None, yaw_rate=None, tolerance=0.1, time_limit=10.0):
        self.set_position(x, y, z, frame, yaw, yaw_rate)
        t0 = self.clock.time()
        while self.clock.time() - t0 < time_limit:
            self.clock.sleep(0.1)
            px, py, pz = self.get_position()[:3]
            if math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - z) ** 2) < tolerance:
                return True
        return False

    def set_position(self, x, y, z, frame, yaw=None, yaw_rate=None):
        self._advance()
        if yaw is None and yaw_rate is None:
            return                                         # как на борту: уставка без курса игнорируется
        if frame != Frame.LOCAL_NED:
            raise NotImplementedError("имитатор: set_position только в LOCAL_NED")
        self.sp = ("pos", x, y, z, yaw)

    def set_velocity(self, vx, vy, vz, frame=Frame.BODY_NED, yaw=None, yaw_rate=0.0):
        self._advance()
        if frame == Frame.BODY_NED:
            c, s = math.cos(self.psi), math.sin(self.psi)
            vn, ve = vx * c - vy * s, vx * s + vy * c
        else:
            c, s = math.cos(self.yaw0), math.sin(self.yaw0)
            vn, ve = vx * c + vy * s, -vx * s + vy * c
        self.sp = ("vel", vn, ve, -vz, yaw_rate, yaw)

    def get_position(self):
        self._advance()
        x, y, z, yaw = self._est()
        g = self.rng.gauss
        return x + g(0, 0.005), y + g(0, 0.005), z + g(0, 0.005), 0.0, 0.0, yaw

    def arm(self):
        self.armed = True

    def disarm(self):
        self.armed = self.flying = False

    def set_flight_mode(self, mode):
        pass

    # ---------- детектор ----------
    def _project(self, pts):
        """Точки сцены (n, e, h) -> пиксели; None, если хоть одна позади камеры."""
        out = []
        c, s = math.cos(self.psi), math.sin(self.psi)
        for n, e, h in pts:
            dn, de = n - self.n, e - self.e
            fwd, right, up = dn * c + de * s, -dn * s + de * c, h - self.h
            if fwd < 0.05:
                return None
            cp, sp = math.cos(self.cam_pitch), math.sin(self.cam_pitch)
            f2, u2 = fwd * cp + up * sp, -fwd * sp + up * cp
            if f2 < 0.05:
                return None
            out.append((self.fw / 2.0 + self.f_px * right / f2, self.fh / 2.0 - self.f_px * u2 / f2))
        return out

    def _box(self, label, pts, class_id, min_visible=0.4):
        p = self._project(pts)
        if p is None:
            return None
        x1, x2 = min(q[0] for q in p), max(q[0] for q in p)
        y1, y2 = min(q[1] for q in p), max(q[1] for q in p)
        area = max(x2 - x1, 1e-6) * max(y2 - y1, 1e-6)
        cx1, cy1, cx2, cy2 = max(x1, 0), max(y1, 0), min(x2, self.fw), min(y2, self.fh)
        if cx2 <= cx1 or cy2 <= cy1 or (cx2 - cx1) * (cy2 - cy1) < min_visible * area:
            return None
        if (cx2 - cx1) < 6 or (cy2 - cy1) < 6 or self.rng.random() > self.det_rate:
            return None
        g = self.rng.gauss
        return Detection(label, min(0.99, max(0.3, g(0.8, 0.06))), cx1 + g(0, 1.5), cy1 + g(0, 1.5),
                         cx2 + g(0, 1.5), cy2 + g(0, 1.5), class_id)

    def get_detections(self):
        self._advance()
        w, out = self.world, []
        if "window" in self.labels and self.n < w.window_n:
            hw, hh = w.window_w / 2, w.window_ht / 2
            b = self._box("window", [(w.window_n, w.window_e - hw, w.window_h - hh), (w.window_n, w.window_e + hw, w.window_h - hh),
                                     (w.window_n, w.window_e - hw, w.window_h + hh), (w.window_n, w.window_e + hw, w.window_h + hh)], 1)
            if b:
                out.append(b)
        if "person" in self.labels and self.n > w.window_n:      # людей ищем за окном
            for (pn, pe, ph, hidden) in w.persons:
                d = math.hypot(pn - self.n, pe - self.e) or 1e-6
                tn, te = -(pe - self.e) / d * 0.22, (pn - self.n) / d * 0.22        # поперёк луча, полширины 22 см
                lo = ph * hidden
                b = self._box("person", [(pn - tn, pe - te, lo), (pn + tn, pe + te, lo),
                                         (pn - tn, pe - te, ph), (pn + tn, pe + te, ph)], 0)
                if b:
                    out.append(b)
        return out
