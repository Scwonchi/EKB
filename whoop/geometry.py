"""Геометрия для программы микродрона: оси, пеленги по рамке детектора, пересечение пеленгов.

Оси
* LOCAL_NED микродрона: x — «север» (N), y — «восток» (E), z — вниз; начало — точка взлёта.
  Курс (yaw) — от оси N по часовой стрелке, радианы.
* Оси ровера от площадки взлёта: f — вперёд по оси ровера, l — влево, высота h — над полом.
  На старте микродрон стоит носом вперёд по оси ровера, поэтому его курс в момент взлёта (psi0)
  задаёт поворот между этими системами.

Только стандартная библиотека: на бортовом компьютере нет numpy.
"""
import math
from itertools import permutations


def wrap(a):
    """Угол в (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def rover_to_ned(f, l, psi0, origin=(0.0, 0.0)):
    """Точка в осях ровера от площадки (вперёд, влево) -> (N, E)."""
    c, s = math.cos(psi0), math.sin(psi0)
    return origin[0] + f * c + l * s, origin[1] + f * s - l * c


def ned_to_rover(n, e, psi0, origin=(0.0, 0.0)):
    """(N, E) -> (вперёд, влево) в осях ровера от площадки."""
    dn, de = n - origin[0], e - origin[1]
    c, s = math.cos(psi0), math.sin(psi0)
    return dn * c + de * s, dn * s - de * c


class Camera(object):
    """Курсовая камера микродрона как камера-обскура: рамка детектора -> углы.

    frame_w, frame_h — размер кадра, в пикселях которого детектор отдаёт рамки; hfov_deg — угол
    обзора по горизонтали; pitch_deg — наклон камеры вверх относительно корпуса.
    Документация микродрона этих чисел не даёт: их измеряют (cam_check.py).
    """

    def __init__(self, frame_w=640, frame_h=640, hfov_deg=90.0, pitch_deg=0.0):
        self.w, self.h = float(frame_w), float(frame_h)
        self.f = (self.w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.pitch = math.radians(pitch_deg)

    def angles(self, px, py):
        """Пиксель -> (азимут вправо от носа, угол места вверх), радианы."""
        az = math.atan2(px - self.w / 2.0, self.f)
        el = math.atan2(self.h / 2.0 - py, self.f) + self.pitch
        return az, el

    def box(self, det):
        """Рамка детектора -> словарь: азимут и угол места центра, угловые ширина и высота,
        доля рамки, упёршаяся в край кадра (cut)."""
        x1, y1, x2, y2 = float(det.x1), float(det.y1), float(det.x2), float(det.y2)
        az, el = self.angles((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        az1, el1 = self.angles(x1, y1)
        az2, el2 = self.angles(x2, y2)
        m = 2.0
        cut = x1 <= m or y1 <= m or x2 >= self.w - m or y2 >= self.h - m
        return {"az": az, "el": el, "aw": abs(az2 - az1), "ah": abs(el1 - el2), "cut": cut,
                "score": float(det.score), "label": det.label}

    def hfov_from_object(self, width_px, width_m, distance_m):
        """Угол обзора (градусы) по предмету известной ширины прямо перед камерой."""
        t = width_m * self.w / (2.0 * distance_m * max(width_px, 1.0))
        return math.degrees(2.0 * math.atan(t))


def intersect(p1, b1, p2, b2, min_angle_deg=6.0):
    """Пересечение двух пеленгов (курс от N по часовой) из точек p1, p2 = (N, E).
    Возвращает ((N, E), дальность от p1, дальность от p2) или None: лучи почти параллельны
    либо пересекаются позади."""
    d1 = (math.cos(b1), math.sin(b1))
    d2 = (math.cos(b2), math.sin(b2))
    det = d1[0] * (-d2[1]) - (-d2[0]) * d1[1]
    if abs(det) < math.sin(math.radians(min_angle_deg)):
        return None
    rn, re = p2[0] - p1[0], p2[1] - p1[1]
    t1 = (rn * (-d2[1]) - (-d2[0]) * re) / det
    t2 = (d1[0] * re - d1[1] * rn) / det
    if t1 <= 0 or t2 <= 0:
        return None
    return (p1[0] + d1[0] * t1, p1[1] + d1[1] * t1), t1, t2


def range_from_height(ah, visible_height_m):
    """Грубая дальность по угловой высоте рамки (если пострадавший виден целиком)."""
    return visible_height_m / (2.0 * math.tan(max(ah, 1e-3) / 2.0))


def triangulate(obs1, obs2, r_min=0.3, r_max=7.0, max_size_ratio=1.7, inside=None):
    """Положения пострадавших по пеленгам из двух точек.

    obs1, obs2 — списки наблюдений: {"n", "e", "bearing", "ah", "score"}.
    Пеленги из разных точек сопоставляются так, чтобы пар было как можно больше, а видимый
    размер пострадавшего (угловая высота x дальность) в обеих точках совпадал: это отличает
    верную пару от случайного пересечения лучей к разным людям.
    inside(n, e) -> bool — необязательная проверка «точка в зоне поиска».
    Возвращает (пары, непарные из obs1, непарные из obs2); пара — словарь с n, e, r1, r2, i, j.
    """
    cand = {}
    for i, a in enumerate(obs1):
        for j, b in enumerate(obs2):
            x = intersect((a["n"], a["e"]), a["bearing"], (b["n"], b["e"]), b["bearing"])
            if x is None:
                continue
            (n, e), r1, r2 = x
            if not (r_min <= r1 <= r_max and r_min <= r2 <= r_max):
                continue
            if inside is not None and not inside(n, e):
                continue
            ratio = (a["ah"] * r1) / max(b["ah"] * r2, 1e-6)
            cost = abs(math.log(max(ratio, 1e-6)))
            if cost > math.log(max_size_ratio):
                continue
            cand[(i, j)] = (cost, {"n": n, "e": e, "r1": r1, "r2": r2, "i": i, "j": j,
                                   "score": min(a["score"], b["score"])})
    best = (0, 0.0, [])
    n1, n2 = len(obs1), len(obs2)
    idx2 = list(range(n2)) + [None] * n1                      # None — «без пары»
    seen = set()
    for perm in permutations(idx2, n1):
        if perm in seen:
            continue
        seen.add(perm)
        pairs, total, ok = [], 0.0, True
        for i, j in enumerate(perm):
            if j is None:
                continue
            c = cand.get((i, j))
            if c is None:
                ok = False
                break
            pairs.append(c[1])
            total += c[0]
        if not ok:
            continue
        if len(pairs) > best[0] or (len(pairs) == best[0] and total < best[1]):
            best = (len(pairs), total, pairs)
    used1 = set(p["i"] for p in best[2])
    used2 = set(p["j"] for p in best[2])
    return (best[2], [o for k, o in enumerate(obs1) if k not in used1],
            [o for k, o in enumerate(obs2) if k not in used2])


def least_squares_point(rays):
    """Точка, ближайшая ко всем лучам: rays — список ((N, E), пеленг). None — лучи параллельны."""
    a11 = a12 = a22 = b1 = b2 = 0.0
    for (pn, pe), brg in rays:
        nn, ne = -math.sin(brg), math.cos(brg)               # нормаль к лучу
        c = nn * pn + ne * pe
        a11 += nn * nn
        a12 += nn * ne
        a22 += ne * ne
        b1 += nn * c
        b2 += ne * c
    det = a11 * a22 - a12 * a12
    if abs(det) < 1e-9:
        return None
    return (b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det


def locate(per_point, r_min=0.3, r_max=7.0, max_size_ratio=1.7, attach_deg=6.0, inside=None):
    """Положения пострадавших по пеленгам из нескольких точек обзора.

    per_point — список по точкам, в каждой список наблюдений {"n", "e", "bearing", "ah", "score"}.
    1. Пары пеленгов из разных точек дают кандидатов (triangulate отсекает случайные пересечения).
    2. Кандидаты берутся от лучшей геометрии к худшей: чем ближе угол между пеленгами к прямому,
       тем точнее пересечение. Каждое наблюдение — не больше чем в одном пострадавшем.
    3. К принятому кандидату добавляются пеленги из остальных точек, которые проходят рядом, и
       положение уточняется по всем лучам сразу.
    Возвращает (найденные, непарные наблюдения); найденный — {"n", "e", "score", "rays"}.
    """
    cands = []
    for i in range(len(per_point)):
        for j in range(i + 1, len(per_point)):
            pairs, _, _ = triangulate(per_point[i], per_point[j], r_min, r_max, max_size_ratio, inside)
            for p in pairs:
                a, b = per_point[i][p["i"]], per_point[j][p["j"]]
                q = abs(math.sin(wrap(a["bearing"] - b["bearing"])))
                cands.append((q, (i, p["i"]), (j, p["j"]), p))
    cands.sort(key=lambda c: -c[0])
    used, found = set(), []
    for q, ka, kb, p in cands:
        if ka in used or kb in used:
            continue
        members = [ka, kb]
        for k, obs in enumerate(per_point):
            if k in (ka[0], kb[0]):
                continue
            best = None
            for m, o in enumerate(obs):
                if (k, m) in used:
                    continue
                want = math.atan2(p["e"] - o["e"], p["n"] - o["n"])
                err = abs(wrap(want - o["bearing"]))
                if err < math.radians(attach_deg) and (best is None or err < best[0]):
                    best = (err, m)
            if best is not None:
                members.append((k, best[1]))
        rays = [((per_point[k][m]["n"], per_point[k][m]["e"]), per_point[k][m]["bearing"]) for k, m in members]
        pt = least_squares_point(rays) or (p["n"], p["e"])
        if inside is not None and not inside(pt[0], pt[1]):
            pt = (p["n"], p["e"])
        used.update(members)
        found.append({"n": pt[0], "e": pt[1], "rays": len(members), "quality": q,
                      "score": max(per_point[k][m]["score"] for k, m in members)})
    rest = [o for k, obs in enumerate(per_point) for m, o in enumerate(obs) if (k, m) not in used]
    return found, rest
