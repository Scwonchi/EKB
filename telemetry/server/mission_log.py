"""
Журнал миссии на ПК: события с отметками времени, фотоснимки, карта обнаружения, отчёт.

Работает с любым потоком кадров [4 байта длины BigEndian][telemetry.v1.Telemetry]: мост
(gateway_bridge.py) подаёт сюда копию байт каждого устройства, ничего не меняя в пересылке.
Поэтому журнал пишется, даже если сервер (iot-go) расширенные поля пока не показывает.

Для каждого устройства и включения — папка missions/<дата>_<время>_<устройство>/:
    events.csv     события: время события, время получения, тип, номер, значение
    map.csv        карта обнаружения (последняя): направление, дистанция, класс, фото
    photos/        снимки photo<N>_det<M>.jpg
    report.html    отчёт для оператора/экспертов: карта, снимки, журнал (открывается без интернета)

Время события на ПК = время включения устройства (оценка по снимкам) + uptime_ms события.
Время включения оценивается как минимум (время приёма − uptime_s) по всем снимкам — точность
~0,2 с; время получения — когда событие впервые пришло на ПК.
"""
import csv
import datetime as dt
import html
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "gen"))
try:
    from telemetry.v1 import telemetry_pb2 as pb
except ImportError:                     # нет protobuf или сгенерированного кода — журнал выключен
    pb = None

MAX_FRAME = 64 * 1024

EVENT_RU = {
    1: "включение", 2: "отделение от носителя", 3: "вход в зону", 4: "начало сканирования",
    5: "остановка", 6: "обнаружен человек", 7: "тревога", 8: "снимок сделан", 9: "тревога передана",
    10: "снимок передан", 11: "отбой тревоги", 12: "сканирование завершено",
}
PHASE_RU = {0: "—", 1: "ожидание", 2: "сканирование", 3: "наблюдение"}
CLASS_RU = {0: "—", 1: "пусто", 2: "движется", 3: "неподвижен", 4: "помеха"}


def fmt_time(t):
    return dt.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3] if t else ""


class FrameSplitter:
    """Собирает кадры [4 байта длины][тело] из кусков потока."""

    def __init__(self):
        self.buf = bytearray()
        self.broken = False

    def feed(self, data: bytes):
        if self.broken:
            return []
        self.buf += data
        out = []
        while len(self.buf) >= 4:
            n = int.from_bytes(self.buf[:4], "big")
            if n > MAX_FRAME:
                self.broken = True             # поток испорчен — дальше не разбираем (пересылка не страдает)
                return out
            if len(self.buf) < 4 + n:
                break
            out.append(bytes(self.buf[4:4 + n]))
            del self.buf[:4 + n]
        return out


class DeviceLog:
    """Журнал одного подключения устройства."""

    def __init__(self, root, device, log):
        self.root, self.device, self.log = root, device, log
        self.split = FrameSplitter()
        self.dir = None
        self.boot = None                       # оценка времени включения устройства (time.time())
        self.last_uptime = None
        self.seen_seq = set()
        self.events = []                       # (wall, recv, seq, type, ref, value, uptime_ms)
        self.photos = {}                       # id -> (file, meta dict)
        self.map = []
        self.scan = None
        self.last_report = 0.0
        self.dirty = False
        self.events_changed = False

    # ---- приём ----
    def feed(self, data: bytes, now: float):
        for body in self.split.feed(data):
            t = pb.Telemetry()
            try:
                t.ParseFromString(body)
            except Exception:
                continue
            self.on_snapshot(t, now)
        if self.dirty and now - self.last_report > 1.0:
            self.write_report()

    def start_dir(self, now):
        stamp = dt.datetime.fromtimestamp(now).strftime("%Y-%m-%d_%H-%M-%S")
        safe = "".join(c if c.isalnum() or c in "-." else "_" for c in self.device)
        self.dir = os.path.join(self.root, f"{stamp}_{safe}")
        os.makedirs(os.path.join(self.dir, "photos"), exist_ok=True)
        self.log(f"журнал миссии: {self.dir}")

    def on_snapshot(self, t, now):
        if self.last_uptime is not None and t.uptime_s + 2 < self.last_uptime:
            self.log(f"{self.device}: устройство перезагрузилось — новый журнал")
            self.__init__(self.root, self.device, self.log)
        self.last_uptime = t.uptime_s
        b = now - t.uptime_s
        self.boot = b if self.boot is None else min(self.boot, b)
        if self.dir is None:
            if not (t.HasField("scan") or t.recent_events or t.detections or t.HasField("photo")):
                return                         # устройство без расширения (поля 17+) — журнала нет
            self.start_dir(now)

        for e in t.recent_events:
            if e.seq in self.seen_seq:
                continue
            self.seen_seq.add(e.seq)
            wall = min(self.boot + e.uptime_ms / 1000.0, now)
            rec = (wall, now, e.seq, e.type, e.ref, e.value, e.uptime_ms)
            self.events.append(rec)
            self.log(f"{self.device}: [{fmt_time(wall)}] {EVENT_RU.get(e.type, e.type)}"
                     + (f" №{e.ref}" if e.ref else "") + (f", {e.value:.1f}" if e.value else ""))
            self.dirty = True
            self.events_changed = True
        if getattr(self, "events_changed", False):
            self.events_changed = False
            self.write_events()

        if t.HasField("photo") and t.photo.id not in self.photos and t.photo.jpeg:
            p = t.photo
            name = f"photo{p.id}_det{p.detection_id}.jpg"
            with open(os.path.join(self.dir, "photos", name), "wb") as f:
                f.write(p.jpeg)
            self.photos[p.id] = (name, {
                "id": p.id, "detection_id": p.detection_id, "time": fmt_time(self.boot + p.taken_uptime_ms / 1000.0),
                "heading": round(p.heading_deg, 1) if p.HasField("heading_deg") else None,
                "relative_heading": round(p.relative_heading_deg, 1), "size": len(p.jpeg),
                "width": p.width, "height": p.height})
            self.log(f"{self.device}: снимок №{p.id} ({len(p.jpeg) // 1024} КБ) -> {name}")
            self.dirty = True

        new_map = [(d.id, round(d.distance_m, 2), round(d.relative_bearing_deg, 1),
                    round(d.bearing_deg, 1) if d.HasField("bearing_deg") else None,
                    d.detection_class, d.stop_index, d.seen_count, d.photo_id,
                    fmt_time(self.boot + d.detected_uptime_ms / 1000.0)) for d in t.detections]
        if new_map != self.map:
            self.map = new_map
            with open(os.path.join(self.dir, "map.csv"), "w", newline="", encoding="utf-8") as f:
                w = csv.writer(f)
                w.writerow(["номер", "дистанция_м", "направление_от_начала_град", "азимут_град", "класс",
                            "остановка", "подтверждён_на_остановках", "фото", "обнаружен"])
                for m in new_map:
                    w.writerow([m[0], m[1], m[2], "" if m[3] is None else m[3], CLASS_RU.get(m[4], m[4]),
                                m[5], m[6], m[7] or "", m[8]])
            self.dirty = True
        if t.HasField("scan"):
            s = (t.scan.phase, t.scan.stops_done, round(t.scan.covered_deg), t.scan.elapsed_s)
            if s != self.scan:
                self.scan = s
                self.dirty = True

    def write_events(self):
        """events.csv целиком: время событий пересчитывается по уточнённому времени включения."""
        with open(os.path.join(self.dir, "events.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["время_события", "время_получения", "устройство", "seq", "событие", "номер", "значение", "uptime_ms"])
            for i, (_, recv, seq, typ, ref, val, up) in enumerate(self.events):
                wall = min(self.boot + up / 1000.0, recv)
                self.events[i] = (wall, recv, seq, typ, ref, val, up)
                w.writerow([fmt_time(wall), fmt_time(recv), self.device, seq, EVENT_RU.get(typ, str(typ)),
                            ref or "", f"{val:.1f}" if val else "", up])

    def close(self):
        if self.dir:
            if self.events:
                self.write_events()
            self.write_report()

    # ---- отчёт ----
    def write_report(self):
        import time
        self.last_report = time.time()
        self.dirty = False
        if not self.dir:
            return
        size, c, r = 360, 180, 150                # карта: круг 6 м
        k = r / 6.0
        svg = [f'<svg viewBox="0 0 {size} {size}" width="{size}" height="{size}" role="img" aria-label="карта обнаружения">',
               f'<circle cx="{c}" cy="{c}" r="{r}" class="ring"/>']
        for m in (2, 4):
            svg.append(f'<circle cx="{c}" cy="{c}" r="{m * k:.0f}" class="ring thin"/>'
                       f'<text x="{c + 3}" y="{c - m * k - 3:.0f}" class="lbl">{m} м</text>')
        svg.append(f'<line x1="{c}" y1="{c}" x2="{c}" y2="{c - r}" class="axis"/>'
                   f'<text x="{c + 4}" y="{c - r + 12}" class="lbl">0° — направление в начале</text>')
        for m in self.map:
            a = math.radians(m[2])
            x, y = c + m[1] * k * math.sin(a), c - m[1] * k * math.cos(a)
            svg.append(f'<line x1="{c}" y1="{c}" x2="{x:.1f}" y2="{y:.1f}" class="ray"/>'
                       f'<circle cx="{x:.1f}" cy="{y:.1f}" r="9" class="person"/>'
                       f'<text x="{x:.1f}" y="{y + 4:.1f}" class="num">{m[0]}</text>')
        svg.append(f'<circle cx="{c}" cy="{c}" r="5" class="dev"/></svg>')

        rows = ""
        for m in self.map:
            az = "" if m[3] is None else f"{m[3]:.0f}°"
            photo = f'<a href="#photo{m[7]}">№{m[7]}</a>' if m[7] in self.photos else "—"
            rows += (f"<tr><td>{m[0]}</td><td>{m[1]:.1f} м</td><td>{m[2]:.0f}°</td><td>{az}</td>"
                     f"<td>{CLASS_RU.get(m[4], m[4])}</td><td>{m[5]}</td><td>{m[6]}</td><td>{m[8]}</td><td>{photo}</td></tr>")
        rows = rows or '<tr><td colspan="9">людей не обнаружено</td></tr>'
        ev = "".join(
            f"<tr><td>{fmt_time(e[0])}</td><td>{fmt_time(e[1])}</td><td>{e[2]}</td>"
            f"<td>{html.escape(EVENT_RU.get(e[3], str(e[3])))}</td><td>{e[4] or ''}</td>"
            f"<td>{f'{e[5]:.1f}' if e[5] else ''}</td></tr>" for e in self.events)
        ph = "".join(
            f'<figure id="photo{pid}"><img src="photos/{html.escape(n)}" alt="снимок {pid}">'
            f"<figcaption>№{pid}, человек №{meta['detection_id'] or '—'}, {meta['time']}, "
            f"курс от начала {meta['relative_heading']:.0f}°"
            + (f", азимут {meta['heading']:.0f}°" if meta['heading'] is not None else "")
            + "</figcaption></figure>"
            for pid, (n, meta) in sorted(self.photos.items()))
        photos_html = f'<div class="photos">{ph}</div>' if ph else "<p>снимков нет</p>"
        scan = ""
        if self.scan:
            scan = (f"<p>Этап: <b>{PHASE_RU.get(self.scan[0], self.scan[0])}</b>, остановок {self.scan[1]}, "
                    f"охват {self.scan[2]}°, время сканирования {self.scan[3]} с</p>")
        doc = f"""<!doctype html><html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Миссия {html.escape(self.device)}</title>
<style>
:root{{--bg:#fff;--fg:#1d2329;--mut:#66707a;--line:#d5dbe0;--acc:#d9480f;--card:#f4f6f8}}
@media (prefers-color-scheme:dark){{:root{{--bg:#15191d;--fg:#e7ebee;--mut:#9aa4ad;--line:#38414a;--acc:#ff8a4c;--card:#1e2328}}}}
body{{background:var(--bg);color:var(--fg);font:15px/1.45 system-ui,sans-serif;margin:0 auto;max-width:1000px;padding:16px}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:17px;margin:24px 0 8px}} p{{margin:4px 0;color:var(--mut)}}
table{{border-collapse:collapse;width:100%;font-size:14px}} td,th{{border-bottom:1px solid var(--line);padding:5px 6px;text-align:left}}
th{{color:var(--mut);font-weight:600}} .wrap{{overflow-x:auto}}
.ring{{fill:none;stroke:var(--line);stroke-width:1.5}} .thin{{stroke-dasharray:3 4}} .axis{{stroke:var(--mut);stroke-dasharray:2 3}}
.ray{{stroke:var(--acc);stroke-width:1.5;opacity:.6}} .person{{fill:var(--acc)}} .dev{{fill:var(--fg)}}
.num{{fill:#fff;font-size:11px;font-weight:700;text-anchor:middle}} .lbl{{fill:var(--mut);font-size:11px}}
.top{{display:flex;gap:24px;flex-wrap:wrap;align-items:flex-start}} .top>div{{flex:1;min-width:280px}}
svg{{max-width:100%;height:auto;background:var(--card);border-radius:8px}}
figure{{margin:0}} .photos{{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:14px}} img{{max-width:100%;border-radius:6px}} figcaption{{color:var(--mut);font-size:13px}}
</style></head><body>
<h1>Миссия: {html.escape(self.device)}</h1>
<p>Отчёт обновлён {dt.datetime.now():%Y-%m-%d %H:%M:%S}. Направления — по часовой от курса в начале сканирования.</p>
{scan}
<div class="top"><div>{''.join(svg)}</div><div><h2>Карта обнаружения</h2><div class="wrap"><table>
<tr><th>№</th><th>Дистанция</th><th>Направление</th><th>Азимут</th><th>Класс</th><th>Остановка</th><th>Подтверждений</th><th>Обнаружен</th><th>Фото</th></tr>
{rows}</table></div></div></div>
<h2>Снимки</h2>{photos_html}
<h2>Журнал событий</h2><div class="wrap"><table>
<tr><th>Время события</th><th>Получено на ПК</th><th>№</th><th>Событие</th><th>Номер</th><th>Значение</th></tr>
{ev}</table></div>
</body></html>"""
        tmp = os.path.join(self.dir, "report.html.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(doc)
        os.replace(tmp, os.path.join(self.dir, "report.html"))


class MissionRecorder:
    def __init__(self, root, log):
        self.root, self.log = root, log
        self.enabled = pb is not None and hasattr(pb, "MissionEvent")
        if pb is None:
            log("журнал миссии выключен: нет пакета protobuf или server/gen (pip install protobuf; easyp generate)")
        elif not self.enabled:
            log("журнал миссии выключен: в схеме telemetry.v1 нет полей миссии (17–20) — используется схема iot-go 1–16")

    def open(self, device):
        """Журнал устройства. После переподключения того же устройства (тот же адрес) журнал
        продолжается в той же папке; перезагрузку устройства DeviceLog замечает сам по uptime."""
        if not self.enabled:
            return None
        if not hasattr(self, "logs"):
            self.logs = {}
        d = self.logs.get(device)
        if d is None:
            d = self.logs[device] = DeviceLog(self.root, device, self.log)
        else:
            d.split = FrameSplitter()          # новое подключение — новый поток кадров
        return d
