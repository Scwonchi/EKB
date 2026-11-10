"""
Сервер телеметрии: принимает устройства ESP32, сообщения protobuf telemetry.v1.Telemetry.

Протокол (docs/device-protocol.md), кадры [длина: 4 байта BigEndian][protobuf] — формат iot-go:
  устройство -> Syn (MAC, оборудование на борту), сервер -> Ack{code: 0},
  дальше устройство ~5 раз в секунду присылает полный снимок Telemetry; сервер молчит.
Идентификатор устройства — его MAC («02:11:22:33:44:55»): тот же после переподключения.
Старые прошивки без Syn (первым кадром сразу Telemetry) тоже принимаются, их идентификатор —
IP-адрес; --require-syn — как новый iot-go: без Syn подключение закрывается.
Тревоги сервер выводит сам: alarm_active false → true — новая тревога, true → false — отбой.

Запуск:
    pip install -r requirements.txt
    easyp generate                                   # из корня проекта, создаёт server/gen/...
    python server.py                                 # устройства подключаются по Wi-Fi к ПК, порт 9000
    python server.py --serial /dev/ttyACM0 --no-tcp  # устройства подключены через ESP-шлюз (USB)

Команды в консоли: list — устройства, quit — выход.
Тревоги дописываются в alarms.jsonl. Страница оператора: http://localhost:8080,
HTTP API: http://localhost:8080/api/v1 (api/openapi.yaml).
"""
import argparse
import asyncio
import datetime as dt
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "gen"))
try:
    from telemetry.v1 import telemetry_pb2 as pb
    from telemetry.v1 import synack_pb2 as sa
except ImportError:
    sys.exit("Нет сгенерированного кода: выполните `easyp generate` в корне проекта")

MAX_FRAME = 64 * 1024      # как у iot-go
IDLE_TIMEOUT = 5           # с без полного кадра — подключение закрывается (как у iot-go, read_timeout)
CLASS = {pb.DETECTION_CLASS_UNSPECIFIED: "unspecified", pb.DETECTION_CLASS_EMPTY: "empty",
         pb.DETECTION_CLASS_MOVING: "moving", pb.DETECTION_CLASS_STATIONARY: "stationary",
         pb.DETECTION_CLASS_INTERFERENCE: "interference"}
CLASS_RU = {"unspecified": "—", "empty": "пусто", "moving": "движется", "stationary": "неподвижен",
            "interference": "помеха"}


def now():
    return dt.datetime.now().strftime("%H:%M:%S")


def now_iso():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def log(msg):
    print(f"[{now()}] {msg}", flush=True)


async def read_frame(reader: asyncio.StreamReader) -> bytes:
    length = int.from_bytes(await reader.readexactly(4), "big")
    if length > MAX_FRAME:
        raise ValueError(f"кадр {length} байт больше допустимого {MAX_FRAME}")
    return await reader.readexactly(length)


def frame(msg) -> bytes:
    body = msg.SerializeToString()
    return len(body).to_bytes(4, "big") + body


def parse_syn(data: bytes):
    """Syn или None. Старый первый кадр (Telemetry) как Syn не разбирается: у поля 1 другой тип."""
    try:
        s = sa.Syn()
        s.ParseFromString(data)
    except Exception:
        return None
    return s if len(s.mac) == 6 else None


def mac_text(mac: bytes) -> str:
    return ":".join(f"{b:02x}" for b in mac)


def validate(t):
    """Те же ограничения, что domain.Telemetry.Validate в iot-go: что не пройдёт там, не пройдёт и здесь."""
    fin = math.isfinite
    if not (fin(t.person_probability) and 0 <= t.person_probability <= 1):
        return "person_probability вне 0..1"
    if t.HasField("position"):
        if not (fin(t.position.lat) and -90 <= t.position.lat <= 90 and fin(t.position.lon) and -180 <= t.position.lon <= 180):
            return "координаты вне диапазона"
    if t.HasField("hdop") and not (fin(t.hdop) and t.hdop >= 0):
        return "hdop отрицательный или не число"
    if t.heading_valid and not t.HasField("heading_deg"):
        return "heading_valid без heading_deg"
    if t.HasField("heading_deg") and not (fin(t.heading_deg) and 0 <= t.heading_deg < 360):
        return f"heading_deg {t.heading_deg} вне [0, 360)"
    if len(t.tracks) > 6:
        return f"целей {len(t.tracks)}, больше 6"
    for i, o in enumerate(t.tracks):
        if not (fin(o.distance_m) and o.distance_m >= 0 and fin(o.angle_deg) and fin(o.speed_mps) and o.speed_mps >= 0
                and fin(o.hit_ratio) and 0 <= o.hit_ratio <= 1):
            return f"tracks[{i}]: неверные значения"
    return None


def heading_of(t):
    """Курс радара из снимка или None (по контракту — только при heading_valid и наличии значения)."""
    return t.heading_deg if t.heading_valid and t.HasField("heading_deg") else None


def offset_point(lat, lon, bearing_deg, dist_m):
    """Точка на расстоянии dist_m по азимуту (для десятков метров плоская Земля точна)."""
    r = 6371000.0
    b = math.radians(bearing_deg)
    return (lat + math.degrees(dist_m * math.cos(b) / r),
            lon + math.degrees(dist_m * math.sin(b) / (r * math.cos(math.radians(lat)))))


def make_alarm(device_id, number, t):
    """Тревога в формате HTTP API (схема Alarm) по снимку, в котором alarm_active стал true."""
    confirmed = [x for x in t.tracks if x.confirmed]
    best = max(confirmed, key=lambda x: x.hit_ratio, default=None)
    heading = heading_of(t)
    dist = best.distance_m if best else None
    angle = best.angle_deg if best else 0.0
    bearing = (heading + angle) % 360 if heading is not None and best else None
    pos = (t.position.lat, t.position.lon) if t.HasField("position") else None
    target = offset_point(pos[0], pos[1], bearing, dist) if pos and bearing is not None else None
    return {
        "id": f"{device_id}-{number}",
        "device_id": device_id,
        "alarm_number": number,
        "active": True,
        "detected_at": now_iso(),
        "cleared_at": None,
        "detection_class": CLASS.get(best.detection_class if best else t.detection_class, "unspecified"),
        "confidence": round(best.hit_ratio if best else t.person_probability, 2),
        "people": t.target_count,
        "distance_m": round(dist, 2) if dist is not None else None,
        "angle_deg": round(angle, 1),
        "radar_heading_deg": round(heading, 1) if heading is not None else None,
        "bearing_deg": round(bearing, 1) if bearing is not None else None,
        "device_position": {"lat": round(pos[0], 7), "lon": round(pos[1], 7)} if pos else None,
        "target_position": {"lat": round(target[0], 7), "lon": round(target[1], 7)} if target else None,
    }


def format_snapshot(t) -> str:
    h = heading_of(t)
    parts = [f"радар {'ok' if t.radar_ok else 'НЕТ'}",
             f"{CLASS_RU.get(CLASS.get(t.detection_class), '?')} p={t.person_probability:.2f}",
             f"GPS {t.position.lat:.6f},{t.position.lon:.6f} ({t.satellites} сп.)" if t.HasField("position") else "GPS нет",
             f"курс {h:.0f}°" if h is not None else "курс —",
             f"людей {t.target_count}, целей {len(t.tracks)}",
             f"Wi-Fi {t.rssi_dbm} дБм, работает {t.uptime_s} с"]
    if t.learning_background:
        parts.append("запоминает фон")
    return " | ".join(parts)


def print_alarm(a):
    lines = [f"!!! ТРЕВОГА {a['id']}: человек {CLASS_RU.get(a['detection_class'], '?')}, "
             f"уверенность {a['confidence'] * 100:.0f}%, людей в зоне {a['people']}"]
    if a["distance_m"] is not None:
        lines.append(f"    дистанция ~{a['distance_m']:.1f} м, угол {a['angle_deg']:+.0f}° от оси радара")
    if a["bearing_deg"] is not None:
        lines.append(f"    азимут на человека {a['bearing_deg']:.0f}° (радар смотрит на {a['radar_heading_deg']:.0f}°)")
    if a["device_position"]:
        p = a["device_position"]
        lines.append(f"    устройство: {p['lat']:.6f}, {p['lon']:.6f}")
    if a["target_position"]:
        p = a["target_position"]
        lines.append(f"    примерная точка человека: {p['lat']:.6f}, {p['lon']:.6f}  "
                     f"https://yandex.ru/maps/?pt={p['lon']:.6f},{p['lat']:.6f}&z=19")
    print("\a" + "\n".join(f"[{now()}] {l}" for l in lines), flush=True)


# ---------- Подключение устройства ----------
class Connection:
    def __init__(self, server, reader, writer):
        self.server, self.reader, self.writer = server, reader, writer
        peer = writer.get_extra_info("peername") or ("?", "?")
        self.address = str(peer[0])
        self.via = str(peer[1])
        self.device_id = self.address          # до Syn — IP; после Syn — MAC
        self.mac = None
        self.onboards = []
        self.last = None
        self.last_log = 0.0
        self.frames = 0
        self.mlog = None                       # журнал миссии (mission_log.py)

    async def handshake(self):
        """Первый кадр: Syn -> ответ Ack. Возвращает первый кадр Telemetry старой прошивки или None."""
        data = await asyncio.wait_for(read_frame(self.reader), IDLE_TIMEOUT)
        syn = parse_syn(data)
        if syn is None:
            if self.server.require_syn:
                raise ValueError("первый кадр не Syn (старая прошивка?), --require-syn")
            return data
        self.mac = mac_text(syn.mac)
        self.device_id = self.mac
        self.onboards = [{"name": o.name, "url": o.url} for o in syn.onboards]
        code = 1 if self.mac in self.server.reject else 0
        self.writer.write(frame(sa.Ack(code=code)))
        await self.writer.drain()
        if code:
            raise ValueError(f"Syn от {self.mac} отклонён (Ack code {code})")
        return None

    async def run(self):
        srv = self.server
        try:
            first = await self.handshake()
        except Exception as e:
            log(f"{self.address}: рукопожатие не прошло: {e!r}")
            self.writer.close()
            return
        srv.attach(self)
        how = f"Syn/Ack, MAC {self.mac}" if self.mac else "старый протокол без Syn"
        log(f"{self.device_id}: подключилось ({self.via}, {how})")
        for o in self.onboards:
            log(f"{self.device_id}:   на борту {o['name']}: {o['url']}")
        if srv.recorder:
            self.mlog = srv.recorder.open(self.device_id)
        try:
            while True:
                if first is not None:
                    data, first = first, None
                else:
                    data = await asyncio.wait_for(read_frame(self.reader), IDLE_TIMEOUT)
                t = pb.Telemetry()
                t.ParseFromString(data)
                bad = validate(t)
                if bad:
                    raise ValueError(f"неверный снимок: {bad}")
                self.frames += 1
                self.last = t
                srv.on_telemetry(self, t)
                if self.mlog:
                    try:
                        self.mlog.on_snapshot(t, time.time())
                        if self.mlog.dirty and time.time() - self.mlog.last_report > 1.0:
                            self.mlog.write_report()
                    except Exception as e:           # журнал не должен ронять подключение
                        log(f"{self.device_id}: ошибка журнала миссии {e!r}")
        except asyncio.TimeoutError:
            log(f"{self.device_id}: нет кадра {IDLE_TIMEOUT} с, отключаю")
        except (asyncio.IncompleteReadError, ConnectionError):
            log(f"{self.device_id}: отключилось")
        except Exception as e:                       # битые данные и т.п.
            log(f"{self.device_id}: ошибка {e!r}, отключаю")
        finally:
            if self.mlog:
                self.mlog.close()
            srv.detach(self)
            try:
                self.writer.close()
            except Exception:
                pass


# ---------- Сервер ----------
class Server:
    def __init__(self, alarms_path, verbose, require_syn=False, reject=()):
        self.require_syn = require_syn
        self.reject = set(reject)      # MAC, которым отвечать Ack{code: 1} (проверка прошивки)
        self.connections = {}          # device_id -> Connection (активное подключение)
        self.alarm_counter = {}        # device_id -> номер последней тревоги
        self.active_alarm = {}         # device_id -> тревога (dict), пока активна
        self.alarms_path = alarms_path
        self.verbose = verbose
        self.hub = None                # HTTP API и страница оператора (web.py), если включены
        self.recorder = None           # журнал миссии (mission_log.py), если включён

    def attach(self, conn):
        old = self.connections.get(conn.device_id)
        if old and old is not conn:
            log(f"{conn.device_id}: новое подключение того же устройства, старое закрываю")
            try:
                old.writer.close()
            except Exception:
                pass
        self.connections[conn.device_id] = conn
        if self.hub:
            self.hub.device_online(conn.device_id, conn.address, conn.mac, conn.onboards)

    def detach(self, conn):
        if self.connections.get(conn.device_id) is conn:
            del self.connections[conn.device_id]
            if self.hub:
                self.hub.device_offline(conn.device_id)
            if conn.device_id in self.active_alarm:
                log(f"{conn.device_id}: связь потеряна при активной тревоге — тревога остаётся до следующих снимков")

    def on_telemetry(self, conn, t):
        dev = conn.device_id
        if self.hub:
            self.hub.telemetry(dev, t)
        loop_t = asyncio.get_running_loop().time()
        if conn.frames == 1 or self.verbose or loop_t - conn.last_log >= 10:
            conn.last_log = loop_t
            log(f"{dev}: {format_snapshot(t)}")

        active = self.active_alarm.get(dev)
        if t.alarm_active and not active:
            n = self.alarm_counter.get(dev, 0) + 1
            self.alarm_counter[dev] = n
            a = make_alarm(dev, n, t)
            self.active_alarm[dev] = a
            print_alarm(a)
            self.save({"event": "alarm", **a})
            if self.hub:
                self.hub.alarm(a)
        elif not t.alarm_active and active:
            active["active"] = False
            active["cleared_at"] = now_iso()
            del self.active_alarm[dev]
            log(f"{dev}: отбой тревоги {active['id']}")
            self.save({"event": "cleared", "id": active["id"], "device_id": dev, "cleared_at": active["cleared_at"]})
            if self.hub:
                self.hub.alarm(active)

    def save(self, rec):
        with open(self.alarms_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")

    async def console(self):
        loop = asyncio.get_running_loop()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                return
            cmd = line.strip()
            if cmd == "quit":
                os._exit(0)
            elif cmd == "list":
                if not self.connections:
                    print("  подключённых устройств нет")
                for c in self.connections.values():
                    print(f"  {c.device_id} ({c.via}): " + (format_snapshot(c.last) if c.last else "снимков ещё не было"))
            elif cmd:
                print("  команды: list | quit")

    async def on_connect(self, reader, writer):
        await Connection(self, reader, writer).run()


async def main():
    ap = argparse.ArgumentParser(description="Сервер телеметрии (telemetry.v1.Telemetry)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--alarms", default="alarms.jsonl", help="файл журнала тревог")
    ap.add_argument("--verbose", action="store_true", help="печатать каждый снимок")
    ap.add_argument("--no-console", action="store_true", help="без команд из консоли")
    ap.add_argument("--serial", help="порт ESP-шлюза, например /dev/ttyACM0 или COM5")
    ap.add_argument("--baud", type=int, default=921600, help="скорость порта шлюза")
    ap.add_argument("--no-tcp", action="store_true", help="не принимать устройства по TCP (только шлюз)")
    ap.add_argument("--http-port", type=int, default=8080, help="порт страницы и API, 0 — выключить")
    ap.add_argument("--require-syn", action="store_true", help="как новый iot-go: первым кадром только Syn")
    ap.add_argument("--reject", action="append", default=[], metavar="MAC",
                    help="отвечать этому MAC Ack{code: 1} (проверка отказа), например 02:11:22:33:44:55")
    ap.add_argument("--log-dir", default="missions", help='папка журналов миссий: события, снимки, report.html ("" — нет)')
    args = ap.parse_args()
    if args.no_tcp and not args.serial:
        sys.exit("Нечего слушать: уберите --no-tcp или укажите --serial")

    srv = Server(args.alarms, args.verbose, args.require_syn, [m.lower() for m in args.reject])
    if args.log_dir:
        from mission_log import MissionRecorder
        srv.recorder = MissionRecorder(args.log_dir, log)
    tasks = []
    if args.http_port:
        from web import WebHub
        srv.hub = WebHub(log)
        tasks.append(asyncio.create_task(srv.hub.serve(args.host, args.http_port)))
    if not args.no_tcp:
        server = await asyncio.start_server(srv.on_connect, args.host, args.port)
        log(f"устройства: TCP {args.host}:{args.port}")
        tasks.append(asyncio.create_task(server.serve_forever()))
    if args.serial:
        from gateway_link import SerialGateway
        tasks.append(asyncio.create_task(SerialGateway(args.serial, args.baud, srv.on_connect, log).run()))
    if not args.no_console:
        tasks.append(asyncio.create_task(srv.console()))
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
