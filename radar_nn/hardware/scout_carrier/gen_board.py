"""
Плата-носитель разведчика (вариант Б) под YD-ESP32-S3 / ESP32-S3-DevKitC-1.
Односторонняя (медь снизу, детали сверху), выводные детали, для ЛУТ или фоторезиста.

Геометрия задаётся здесь; скрипт сам проверяет зазоры и связность цепей и выпускает:
    out_<шаг>/copper_lut.svg       медь для ЛУТ/плёнки: чёрное — медь, вид сверху (НЕ зеркалить)
    out_<шаг>/copper_negative.svg  то же в негативе — для негативного фоторезиста (плёночного)
    out_<шаг>/assembly.svg         расстановка деталей и отверстий (вид сверху)
    out_<шаг>/print.pdf            всё это на листах A4 в масштабе 1:1
    out_<шаг>/gerber/              Gerber (нижний слой, контур) и Excellon — для станка или завода

    python gen_board.py            # оба варианта: ряды через 25,4 мм (YD) и 22,86 мм (DevKitC-1)

Все размеры в мм, ось y вниз, вид сверху (со стороны деталей).
"""
import math
import os
import subprocess
import sys

PITCH = 2.54
PAD_LONG, PAD_SHORT = 2.6, 1.8       # овальная площадка под штырь
DRILL = 1.0                          # отверстие под штырь 2,54 мм
MOUNT_DRILL = 3.2                    # крепёж M3
W_SIG, W_PWR = 0.8, 1.2              # ширина дорожек: сигнал / питание
MIN_CLEAR = 0.5                      # минимальный зазор медь—медь для домашнего травления
EDGE_CLEAR = 1.0                     # медь — край платы


class Board:
    def __init__(self, name):
        self.name = name
        self.pads = []      # dict: x, y, horiz (вытянута по x), net, label, comp
        self.tracks = []    # dict: pts [(x,y)...], w, net
        self.holes = []     # (x, y, d)
        self.parts = []     # dict: ref, x0, y0, x1, y1, label, kind
        self.texts = []     # подписи на сборочном чертеже: (x, y, text, size, anchor)
        self.outline = None
        self.text_at = None

    def header(self, ref, x, y, n, vertical, nets, labels, kind="female", body=None, label="", size=None):
        """Ряд штырей с шагом 2,54: vertical — сверху вниз, иначе слева направо.
        size — (ширина, высота) площадки; по умолчанию овал, вытянутый поперёк ряда."""
        for i in range(n):
            px, py = (x, y + i * PITCH) if vertical else (x + i * PITCH, y)
            w, h = size or ((PAD_LONG, PAD_SHORT) if vertical else (PAD_SHORT, PAD_LONG))
            self.pads.append(dict(x=px, y=py, w=w, h=h, net=nets[i], label=labels[i], comp=ref, num=i + 1))
        if body:
            self.parts.append(dict(ref=ref, box=body, label=label, kind=kind))

    def pad(self, comp, num):
        for p in self.pads:
            if p["comp"] == comp and p["num"] == num:
                return p["x"], p["y"]
        raise KeyError((comp, num))

    def track(self, net, w, *pts):
        self.tracks.append(dict(net=net, w=w, pts=list(pts)))


# ---------- Геометрия ----------
def seg_dist(p1, p2, q1, q2):
    """Расстояние между отрезками p1p2 и q1q2."""
    def d_ps(p, a, b):
        ax, ay = a; bx, by = b; px, py = p
        dx, dy = bx - ax, by - ay
        L = dx * dx + dy * dy
        t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
        return math.hypot(px - ax - t * dx, py - ay - t * dy)

    def inter(a, b, c, d):
        def cr(o, p, q): return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
        d1, d2, d3, d4 = cr(c, d, a), cr(c, d, b), cr(a, b, c), cr(a, b, d)
        return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 * d2 < 0 and d3 * d4 < 0
    if inter(p1, p2, q1, q2):
        return 0.0
    return min(d_ps(p1, q1, q2), d_ps(p2, q1, q2), d_ps(q1, p1, p2), d_ps(q2, p1, p2))


def capsules(b):
    """Вся медь как «капсулы»: (отрезок, радиус, цепь, описание)."""
    out = []
    for p in b.pads:
        r = min(p["w"], p["h"]) / 2
        h = abs(p["w"] - p["h"]) / 2
        a, c = ((p["x"] - h, p["y"]), (p["x"] + h, p["y"])) if p["w"] > p["h"] else ((p["x"], p["y"] - h), (p["x"], p["y"] + h))
        out.append((a, c, r, p["net"], f'{p["comp"]}.{p["num"]}({p["label"]})'))
    for t in b.tracks:
        for a, c in zip(t["pts"], t["pts"][1:]):
            out.append((a, c, t["w"] / 2, t["net"], f'дорожка {t["net"]} {a}->{c}'))
    return out


def drc(b):
    caps = capsules(b)
    errors = []
    minc = 99.0
    # 1) зазоры между разными цепями
    for i in range(len(caps)):
        for j in range(i + 1, len(caps)):
            a, c = caps[i], caps[j]
            if a[3] == c[3] and a[3] is not None:
                continue
            d = seg_dist(a[0], a[1], c[0], c[1]) - a[2] - c[2]
            minc = min(minc, d)
            if d < MIN_CLEAR:
                errors.append(f"зазор {d:.2f} мм < {MIN_CLEAR}: {a[4]} и {c[4]}")
    # 2) медь у края и у крепёжных отверстий
    x0, y0, x1, y1 = b.outline
    def in_pad(px, py, net):        # конец дорожки внутри площадки своей цепи — край проверяет площадка
        return any(q["net"] == net and math.hypot(q["x"] - px, q["y"] - py) < 0.01 for q in b.pads)
    for a in caps:
        lim = 0.3 if a[4].startswith(("J1.", "J3.", "XR.")) else EDGE_CLEAR
        for (px, py) in (a[0], a[1]):
            if a[4].startswith("дорожка") and in_pad(px, py, a[3]):
                continue
            e = min(px - x0, x1 - px, py - y0, y1 - py) - a[2]
            if e < lim:
                errors.append(f"до края {e:.2f} мм: {a[4]}")
        for hx, hy, hd in b.holes:
            if hd > 2:
                d = seg_dist(a[0], a[1], (hx, hy), (hx, hy)) - a[2] - hd / 2 - 1.5   # под головку винта
                if d < 0:
                    errors.append(f"медь под головкой винта у ({hx},{hy}): {a[4]}")
    # 3) связность: каждая цепь — один кусок меди, все её площадки соединены
    parent = list(range(len(caps)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(caps)):
        for j in range(i + 1, len(caps)):
            if caps[i][3] == caps[j][3] and caps[i][3] is not None:
                if seg_dist(caps[i][0], caps[i][1], caps[j][0], caps[j][1]) <= caps[i][2] + caps[j][2] - 0.05:
                    parent[find(i)] = find(j)
    # выводы платы ESP32-S3 с одной цепью (3V3, GND) соединены внутри неё самой
    idx = {}
    for i, p in enumerate(b.pads):
        if p["comp"] in ("J1", "J3") and p["net"]:
            idx.setdefault(p["net"], []).append(i)
    for net, ii in idx.items():
        for i in ii[1:]:
            parent[find(i)] = find(ii[0])
    nets = {}
    for i, c in enumerate(caps):
        if c[3] is not None:
            nets.setdefault(c[3], set()).add(find(i))
    report = []
    for net, groups in sorted(nets.items()):
        npads = sum(1 for p in b.pads if p["net"] == net)
        if len(groups) != 1:
            errors.append(f"цепь {net} разорвана на {len(groups)} части")
        report.append(f"{net}: {npads} площадок")
    return errors, minc, report


# ---------- Плата ----------
ROUND = 1.8                              # круглая площадка рядов ESP32-S3 (у края платы)


def build(row):
    """row — расстояние между рядами штырей платы ESP32-S3: 25.4 (YD-ESP32-S3) или 22.86 (DevKitC-1).

    Плата-носитель по контуру самой ESP32-S3: ширина = row + 2,6 мм, длина 61,5 мм.
    GY-521 и компас лежат на своих штырях между рядами, ПОД платой ESP32-S3 (там ~11 мм:
    гнездо 8,5 мм + пластик штырей ESP32-S3), радар — угловой разъём у края со стороны USB.
    """
    b = Board(f"{row:.2f}")
    J1X, Y0 = 1.3, 3.5                   # левый ряд и вывод 1
    J3X = J1X + row
    CX = (J1X + J3X) / 2                 # середина между рядами

    def yp(n): return Y0 + (n - 1) * PITCH

    # Ряды платы ESP32-S3 (вид сверху, USB внизу). Порядок как у DevKitC-1 и YD-ESP32-S3.
    j1 = ["3V3", "3V3", "RST", "4", "5", "6", "7", "15", "16", "17", "18", "8", "3", "46", "9", "10",
          "11", "12", "13", "14", "5V", "GND"]
    j3 = ["GND", "TX", "RX", "1", "2", "42", "41", "40", "39", "38", "37", "36", "35", "0", "45", "48",
          "47", "21", "20", "19", "GND", "GND"]
    net1 = {1: "3V3", 2: "3V3", 10: "SCL", 11: "SDA", 19: "GPIO13", 20: "GPIO14", 21: "5V", 22: "GND"}
    net3 = {1: "GND", 21: "GND", 22: "GND"}
    b.header("J1", J1X, Y0, 22, True, [net1.get(i + 1) for i in range(22)], j1, size=(ROUND, ROUND))
    b.header("J3", J3X, Y0, 22, True, [net3.get(i + 1) for i in range(22)], j3, size=(ROUND, ROUND))
    b.parts.append(dict(ref="U1", box=(J1X - 1.3, Y0 - 3.5, J3X + 1.3, Y0 + 58.0),
                        label=("YD-ESP32-S3" if row > 24 else "ESP32-S3-DevKitC-1") + " (сверху, USB вниз)", kind="module"))

    # GY-521: ряд из 8 штырей поперёк платы на уровне вывода 2, модуль лежит корпусом вниз (к USB)
    gx = [CX + (k - 3.5) * PITCH for k in range(8)]
    GY = yp(2)
    b.header("XG", gx[0], GY, 8, False, ["3V3", "GND", "SCL", "SDA", None, None, None, None],
             ["VCC", "GND", "SCL", "SDA", "XDA", "XCL", "AD0", "INT"], size=(ROUND, 2.4),
             body=(CX - 10.5, GY - 1.6, CX + 10.5, GY + 15.0), label="GY-521 (MPU-6050)")
    # Компас: те же выводы точно под выводами GY-521, на уровне вывода 9
    CYc = yp(9)
    b.header("XC", gx[0], CYc, 5, False, ["3V3", "GND", "SCL", "SDA", None],
             ["VCC", "GND", "SCL", "SDA", "DRDY"], size=(ROUND, 2.4),
             body=(gx[0] - 2.0, CYc - 1.6, gx[4] + 2.0, CYc + 16.0), label="компас (по желанию)")
    VX, GX, SCX, SDX = gx[0], gx[1], gx[2], gx[3]
    b.track("3V3", 1.0, (J1X, yp(2)), (VX, GY))                                   # 3V3 -> VCC
    b.track("GND", 1.0, (J3X, yp(1)), (J3X - 1.6, Y0 - 1.5), (GX, Y0 - 1.5), (GX, GY))
    for x, net, w in ((VX, "3V3", 1.0), (GX, "GND", 1.0), (SCX, "SCL", W_SIG), (SDX, "SDA", W_SIG)):
        b.track(net, w, (x, GY), (x, CYc))                                         # GY-521 -> компас
    b.track("SCL", W_SIG, (SCX, CYc), (SCX, yp(10)), (J1X, yp(10)))                # GPIO17
    b.track("SDA", W_SIG, (SDX, CYc), (SDX, yp(11)), (J1X, yp(11)))                # GPIO18

    # Радар: угловой разъём 1×4 у края со стороны USB, штыри наружу
    YR = Y0 + 56.4
    rx = [J1X + 3.0 + k * PITCH for k in range(4)]
    b.header("XR", rx[0], YR, 4, False, ["GND", "5V", "GPIO14", "GPIO13"],
             ["GND", "5V", "RX рад.", "TX рад."], kind="male", size=(ROUND, 2.2),
             body=(rx[0] - 1.5, YR - 1.4, rx[3] + 1.5, YR + 1.6), label="LD2450 (угловой)")
    b.track("GND", 1.0, (J1X, yp(22)), (rx[0], yp(22)), (rx[0], YR))
    b.track("5V", 1.0, (J1X, yp(21)), (rx[1], yp(21)), (rx[1], YR))
    b.track("GPIO14", W_SIG, (J1X, yp(20)), (rx[2], yp(20)), (rx[2], YR))          # TX ESP -> RX радара
    b.track("GPIO13", W_SIG, (J1X, yp(19)), (rx[3], yp(19)), (rx[3], YR))          # TX радара -> RX ESP

    b.outline = (J1X - 1.3, 0.0, J3X + 1.3, Y0 + 58.0)
    b.text_at = (CX, yp(16) + 1.0)
    for p in b.pads:
        b.holes.append((p["x"], p["y"], DRILL))
    return b


# ---------- Вывод: SVG ----------
def pad_path(p, grow=0.0):
    w, h = p["w"] + 2 * grow, p["h"] + 2 * grow
    r = min(w, h) / 2
    return f'<rect x="{p["x"] - w / 2:.3f}" y="{p["y"] - h / 2:.3f}" width="{w:.3f}" height="{h:.3f}" rx="{r:.3f}"/>'


def track_path(t, grow=0.0):
    d = "M" + " L".join(f"{x:.3f},{y:.3f}" for x, y in t["pts"])
    return f'<path d="{d}" fill="none" stroke-width="{t["w"] + 2 * grow:.3f}" stroke-linecap="round" stroke-linejoin="round"/>'


def copper_svg(b, negative=False):
    x0, y0, x1, y1 = b.outline
    W, H = x1 - x0, y1 - y0
    fg, bg = ("#fff", "#000") if negative else ("#000", "#fff")
    s = [f'<g transform="translate({-x0},{-y0})">',
         f'<rect x="{x0}" y="{y0}" width="{W}" height="{H}" fill="{bg}"/>' if negative else
         f'<rect x="{x0 + 0.075}" y="{y0 + 0.075}" width="{W - 0.15}" height="{H - 0.15}" fill="none" stroke="#000" stroke-width="0.15"/>',
         f'<g fill="{fg}" stroke="none">']
    s += [pad_path(p) for p in b.pads]
    s += [f'<g fill="none" stroke="{fg}">' + "".join(track_path(t) for t in b.tracks) + "</g>"]
    s.append("</g>")
    # отверстия-кернения: белая точка в центре площадки помогает сверлить
    s.append(f'<g fill="{bg}">' + "".join(f'<circle cx="{hx}" cy="{hy}" r="{0.35 if d < 2 else d / 2}"/>'
                                         for hx, hy, d in b.holes) + "</g>")
    if negative:   # крепёжные отверстия без меди — обвести
        s.append("".join(f'<circle cx="{hx}" cy="{hy}" r="{d / 2}" fill="none" stroke="#fff" stroke-width="0.2"/>'
                         for hx, hy, d in b.holes if d > 2))
    else:
        s.append("".join(f'<circle cx="{hx}" cy="{hy}" r="{d / 2}" fill="none" stroke="#000" stroke-width="0.2"/>'
                         for hx, hy, d in b.holes if d > 2))
    # надпись медью, зеркальная в виде сверху — на готовой плате со стороны меди читается нормально
    tx, ty = b.text_at
    s.append(f'<text x="0" y="0" transform="translate({tx},{ty}) scale(-1,1)" font-family="Arial" font-weight="bold" '
             f'font-size="1.9" text-anchor="middle" fill="{fg}">SCOUT-S3</text>'
             f'<text x="0" y="0" transform="translate({tx},{ty + 2.4}) scale(-1,1)" font-family="Arial" '
             f'font-size="1.6" text-anchor="middle" fill="{fg}">{b.name}</text>')
    s.append("</g>")
    return W, H, "\n".join(s)


def assembly_svg(b):
    """Вид сверху с полями 12 мм: подписи выводов ESP32-S3 — снаружи платы."""
    x0, y0, x1, y1 = b.outline
    M = 12.0
    W, H = x1 - x0 + 2 * M, y1 - y0 + 2 * M
    blue, ink = "#1c7ed6", "#1d2329"
    s = [f'<g transform="translate({M - x0},{M - y0})" font-family="Arial">',
         f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="{y1 - y0}" fill="#fff" stroke="#000" stroke-width="0.25"/>',
         '<g fill="none" stroke="#d5dade">' + "".join(track_path(t) for t in b.tracks) + "</g>"]
    for part in b.parts:
        if part["kind"] == "module":
            continue
        bx0, by0, bx1, by1 = part["box"]
        s.append(f'<rect x="{bx0}" y="{by0}" width="{bx1 - bx0}" height="{by1 - by0}" fill="{blue}" fill-opacity="0.06" '
                 f'stroke="{blue}" stroke-width="0.25" rx="0.6"/>')
    s.append('<g fill="#fff" stroke="#000" stroke-width="0.2">' + "".join(pad_path(p) for p in b.pads) + "</g>")
    s.append('<g fill="#000">' + "".join(f'<circle cx="{p["x"]}" cy="{p["y"]}" r="0.45"/>' for p in b.pads) + "</g>")
    for p in b.pads:
        if p["comp"] == "J1":
            s.append(f'<text x="{p["x"] - 1.6}" y="{p["y"] + 0.5}" font-size="1.3" text-anchor="end" fill="{ink}">{p["label"]}</text>')
        elif p["comp"] == "J3":
            s.append(f'<text x="{p["x"] + 1.6}" y="{p["y"] + 0.5}" font-size="1.3" fill="{ink}">{p["label"]}</text>')
        elif p["comp"] == "XR":
            ly = y1 + 1.2
            s.append(f'<text x="{p["x"] + 0.45}" y="{ly}" font-size="1.2" text-anchor="end" fill="{ink}" '
                     f'transform="rotate(-90 {p["x"] + 0.45} {ly})">{p["label"]}</text>')
        else:
            ly = p["y"] + 1.6
            s.append(f'<text x="{p["x"] + 0.42}" y="{ly}" font-size="1.05" text-anchor="end" fill="{ink}" '
                     f'transform="rotate(-90 {p["x"] + 0.42} {ly})">{p["label"]}</text>')
    for part in b.parts:
        bx0, by0, bx1, by1 = part["box"]
        cx = (bx0 + bx1) / 2
        if part["kind"] == "module":
            s.append(f'<rect x="{bx0 - 0.4}" y="{by0 - 0.4}" width="{bx1 - bx0 + 0.8}" height="{by1 - by0 + 0.8}" fill="none" '
                     f'stroke="{blue}" stroke-width="0.3" stroke-dasharray="1.2 0.8" rx="1"/>'
                     f'<text x="{cx}" y="{y0 - 2}" font-size="1.6" text-anchor="middle" fill="{blue}">{part["label"]}</text>')
        elif part["ref"] == "XR":
            s.append(f'<text x="{bx1 + 1.0}" y="{by0 + 2.2}" font-size="1.5" fill="{blue}">{part["label"]}</text>')
        else:
            name, _, note = part["label"].partition(" (")
            s.append(f'<text x="{cx}" y="{by0 + 9}" font-size="1.5" text-anchor="middle" fill="{blue}">{name}</text>'
                     + (f'<text x="{cx}" y="{by0 + 10.8}" font-size="1.2" text-anchor="middle" fill="{blue}">({note}</text>' if note else "")
                     + f'<text x="{cx}" y="{by0 + 12.8}" font-size="1.1" text-anchor="middle" fill="{blue}">лёжа под ESP32-S3,</text>'
                     f'<text x="{cx}" y="{by0 + 14.3}" font-size="1.1" text-anchor="middle" fill="{blue}">корпус сюда</text>')
    s.append("</g>")
    return W, H, "\n".join(s)


def svg_doc(W, H, body):
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">\n'
            f'{body}\n</svg>\n')


# ---------- Вывод: Gerber и Excellon ----------
def gerber(b, path_dir):
    os.makedirs(path_dir, exist_ok=True)
    x0, y0, x1, y1 = b.outline
    H = y1

    def c(v): return int(round(v * 1e6))
    def pt(x, y): return f"X{c(x)}Y{c(H - y)}"
    ap = {}
    lines = ["G04 SCOUT-S3 нижний слой меди*", "%FSLAX46Y46*%", "%MOMM*%", "%LPD*%"]
    dcode = 10

    def aperture(key, spec):
        nonlocal dcode
        if key not in ap:
            ap[key] = dcode
            lines.append(f"%ADD{dcode}{spec}*%")
            dcode += 1
        return ap[key]
    for p in b.pads:
        w, h = p["w"], p["h"]
        aperture((w, h), f"C,{w:.3f}" if w == h else f"O,{w:.3f}X{h:.3f}")
    for t in b.tracks:
        aperture(("c", t["w"]), f"C,{t['w']:.3f}")
    body = []
    for p in b.pads:
        body += [f"D{ap[(p['w'], p['h'])]}*", f"{pt(p['x'], p['y'])}D03*"]
    for t in b.tracks:
        body.append(f"D{ap[('c', t['w'])]}*")
        body.append(f"{pt(*t['pts'][0])}D02*")
        body += [f"{pt(*q)}D01*" for q in t["pts"][1:]]
    lines += ["G01*"] + body + ["M02*"]
    with open(os.path.join(path_dir, "scout_s3-B_Cu.gbl"), "w", encoding="ascii", errors="replace") as f:
        f.write("\n".join(l.replace("нижний слой меди", "bottom copper") for l in lines) + "\n")
    # контур
    ol = ["G04 SCOUT-S3 board outline*", "%FSLAX46Y46*%", "%MOMM*%", "%ADD10C,0.100*%", "D10*", "G01*",
          f"{pt(x0, y0)}D02*", f"{pt(x1, y0)}D01*", f"{pt(x1, y1)}D01*", f"{pt(x0, y1)}D01*", f"{pt(x0, y0)}D01*", "M02*"]
    with open(os.path.join(path_dir, "scout_s3-Edge_Cuts.gm1"), "w") as f:
        f.write("\n".join(ol) + "\n")
    # сверловка
    tools = sorted({d for _, _, d in b.holes})
    ex = ["M48", "; SCOUT-S3 drill", "METRIC,TZ"] + [f"T{i + 1}C{d:.3f}" for i, d in enumerate(tools)] + ["G90", "%", "G05"]
    for i, d in enumerate(tools):
        ex.append(f"T{i + 1}")
        ex += [f"X{x:.3f}Y{H - y:.3f}" for x, y, dd in b.holes if dd == d]
    ex.append("M30")
    with open(os.path.join(path_dir, "scout_s3.drl"), "w") as f:
        f.write("\n".join(ex) + "\n")


# ---------- Листы для печати ----------
def print_html(b, copper, negative, assembly):
    aW, aH, aBody = assembly
    big = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{aW * 2}mm" height="{aH * 2}mm" viewBox="0 0 {aW} {aH}" '
           f'style="display:inline-block;vertical-align:top">{aBody}</svg>')
    def sheet(title, notes, img, copies=1):
        W, H, body = img
        one = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}" '
               f'style="display:inline-block;margin:0 8mm 6mm 0">{body}</svg>')
        return f"""<section class="sheet"><h1>{title}</h1><p>{notes}</p>
{one * copies}
<div class="scale"><div class="bar"></div><span>50 мм — проверьте линейкой: если не 50 мм, печатайте в масштабе 100%, без «по размеру страницы»</span></div></section>"""
    return f"""<!doctype html><html lang="ru"><head><meta charset="utf-8"><title>SCOUT-S3 {b.name}</title><style>
@page {{ size: A4; margin: 12mm; }}
body {{ margin: 0; font-family: Arial, sans-serif; color: #000; }}
.sheet {{ page-break-after: always; }}
h1 {{ font-size: 14pt; margin: 0 0 2mm; }} p {{ font-size: 9pt; margin: 0 0 4mm; }}
.scale {{ margin-top: 4mm; font-size: 8pt; display: flex; align-items: center; gap: 3mm; }}
.bar {{ width: 50mm; height: 2mm; background: #000; }}
</style></head><body>
{sheet(f"Медь для ЛУТ / позитивного фоторезиста — ряды {b.name} мм",
       "Чёрное — медь. Вид сверху, со стороны деталей; медь на нижней стороне. <b>Печатать как есть, НЕ зеркалить</b>: при переносе тонером на нижнюю сторону рисунок отразится сам. "
       "Надпись SCOUT-S3 на листе зеркальная — так и надо, на готовой плате она читается. Лазерный принтер, максимальная плотность, без экономии тонера. Четыре копии — про запас.", copper, 4)}
{sheet(f"Негатив для плёночного (негативного) фоторезиста — ряды {b.name} мм",
       "Белое (прозрачное на плёнке) — медь. Печать на прозрачной плёнке тонером к плате, не зеркалить. Для позитивного фоторезиста — первый лист. Четыре копии — про запас.", negative, 4)}
{sheet(f"Сборка (вид сверху) — ряды {b.name} мм",
       "Слева 1:1 — приложите ESP32-S3 штырями для проверки; справа — увеличено. Детали ставятся сверху, паяются снизу, все отверстия Ø1,0 мм. "
       "Серым — дорожки снизу. Сначала впаять GY-521 и компас (лёжа на своих штырях, корпусом к USB), потом гнёзда 1×22 для ESP32-S3, "
       "угловой разъём радара — штырями наружу.", assembly) .replace("</svg>", "</svg>" + big, 1)}
</body></html>"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ok = True
    for row in (25.4, 22.86):
        b = build(row)
        errors, minc, report = drc(b)
        tag = "yd_25.40" if row > 24 else "devkitc_22.86"
        out = os.path.join(here, f"out_{tag}")
        os.makedirs(out, exist_ok=True)
        x0, y0, x1, y1 = b.outline
        print(f"== ряды {row} мм: плата {x1 - x0:.1f} × {y1 - y0:.1f} мм, площадок {len(b.pads)}, отверстий {len(b.holes)}, "
              f"дорожек {len(b.tracks)}, мин. зазор {minc:.2f} мм")
        print("   цепи: " + "; ".join(report))
        for e in errors:
            print("   ОШИБКА", e)
        ok &= not errors
        cu, neg, asm = copper_svg(b), copper_svg(b, True), assembly_svg(b)
        for name, (W, H, body) in (("copper_lut", cu), ("copper_negative", neg), ("assembly", asm)):
            with open(os.path.join(out, name + ".svg"), "w", encoding="utf-8") as f:
                f.write(svg_doc(W, H, body))
        with open(os.path.join(out, "print.html"), "w", encoding="utf-8") as f:
            f.write(print_html(b, cu, neg, asm))
        gerber(b, os.path.join(out, "gerber"))
    print("ИТОГ:", "ошибок нет" if ok else "ЕСТЬ ОШИБКИ")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
