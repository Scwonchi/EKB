"""
Имитатор устройств с радаром LD2450 — для проверки сервера и страницы оператора без железа.
Как настоящая прошивка: при подключении Syn (свой MAC 02:51:4d:00:00:0N и камера) -> Ack,
затем снимки telemetry.v1.Telemetry ~5 раз в секунду в кадрах [4 байта длины BigEndian][protobuf].
Работает и с server.py, и с iot-go (с Syn/Ack и старым).

    python tools/sim_device.py                  # одно устройство, сервер на localhost:9000
    python tools/sim_device.py --count 2        # два устройства
    python tools/sim_device.py --host 192.168.1.10

Сервер с Syn/Ack различает устройства по MAC. Для старого протокола (--handshake off) —
по IP-адресу, поэтому локально каждое имитируемое устройство подключается со своего адреса
127.0.0.2, 127.0.0.3, ... (Linux, Windows).

Сценарий (повторяется каждые 60 с): человек заходит в зону и идёт поперёк,
через ~2 с подтверждается — тревога; второй человек сидит неподвижно; иногда
мелькает ложная точка; через ~45 с все уходят, через 5 с — отбой.
"""
import argparse
import asyncio
import math
import os
import random
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "server", "gen"))
from telemetry.v1 import telemetry_pb2 as pb  # noqa: E402
from telemetry.v1 import synack_pb2 as sa  # noqa: E402

CLEAR_S = 5          # как на устройстве: отбой через 5 с без людей


def frame(body: bytes) -> bytes:
    """[длина тела: 4 байта BigEndian][тело] — формат iot-go."""
    return len(body).to_bytes(4, "big") + body


class SimDevice:
    def __init__(self, idx, host, port, handshake="auto", fly=False):
        self.idx, self.host, self.port = idx, host, port
        self.fly = fly
        self.handshake = handshake
        self.mac = bytes([0x02, 0x51, 0x4D, 0x00, 0x00, idx + 1])   # постоянный MAC имитатора
        self.heading = (90 + 120 * idx) % 360
        self.lat, self.lon = 55.7512 + 0.0004 * idx, 37.6184 + 0.0006 * idx
        self.t0 = time.time() - 20 * idx
        self.boot = time.time()
        self.alarm = False
        self.last_person = 0.0
        self.local = (f"127.0.0.{idx + 2}", 0) if host in ("127.0.0.1", "localhost") else None

    @property
    def name(self):
        return f"sim{self.idx}" + (f" ({self.local[0]})" if self.local else "")

    def people(self, t):
        """Цели: (id, дистанция, угол, скорость, подтверждена, класс, видимость)."""
        s = (t - self.t0) % 60
        out = []
        if 5 < s < 45:                                     # идёт поперёк зоны
            x = -2.5 + (s - 5) * 0.12
            y = 3.2 + 0.4 * math.sin(s / 4)
            out.append((1, math.hypot(x, y), math.degrees(math.atan2(x, y)), 0.6, s > 7.5,
                        pb.DETECTION_CLASS_MOVING, 0.9))
        if 15 < s < 40:                                    # сидит неподвижно
            out.append((2, 1.9 + random.gauss(0, 0.02), -28 + random.gauss(0, 0.5), 0.02, s > 17.5,
                        pb.DETECTION_CLASS_STATIONARY, 0.75))
        if random.random() < 0.15:                         # ложная точка
            out.append((100 + int(t * 5) % 50, random.uniform(0.8, 5.5), random.uniform(-55, 55), 0.0, False,
                        pb.DETECTION_CLASS_UNSPECIFIED, 0.2))
        return out

    def flight(self, t):
        """--fly: облёт помещения по прямоугольнику 4×3 м со скоростью 0,4 м/с и разворотами на месте.
        Возвращает (широта, долгота, курс) — как разведчик с одометрией: точка старта + метры."""
        sides = [(4.0, 0), (3.0, 90), (4.0, 180), (3.0, 270)]     # длина, курс от «вперёд на старте»
        az0 = (30 + 90 * self.idx) % 360                           # куда смотрел дрон на старте
        per = sum(L / 0.4 + 3 for L, _ in sides)                   # +3 с на разворот
        s = (t - self.t0) % per
        x = y = 0.0
        head = 0
        for L, h in sides:
            dur = L / 0.4
            head = h
            if s < 3:                                              # разворот на месте
                break
            s -= 3
            k = min(s, dur) * 0.4
            x += k * math.sin(math.radians(h)); y += k * math.cos(math.radians(h))
            if s < dur:
                break
            s -= dur
        a = math.radians(az0)
        north = y * math.cos(a) - x * math.sin(a)
        east = y * math.sin(a) + x * math.cos(a)
        lat = self.lat + math.degrees(north / 6371000)
        lon = self.lon + math.degrees(east / (6371000 * math.cos(math.radians(self.lat))))
        return lat, lon, (az0 + head) % 360

    def snapshot(self, t):
        ppl = self.people(t)
        confirmed = [p for p in ppl if p[4]]
        if confirmed:
            self.last_person = t
            self.alarm = True
        elif self.alarm and t - self.last_person > CLEAR_S:
            self.alarm = False
        best = max(confirmed, key=lambda p: p[6], default=None)
        lat, lon, heading = self.flight(t) if self.fly else (self.lat, self.lon, self.heading)
        m = pb.Telemetry(
            firmware_version=3, uptime_s=int(t - self.boot), radar_ok=True,
            detection_class=best[5] if best else pb.DETECTION_CLASS_EMPTY,
            person_probability=best[6] if best else 0.0,
            position=pb.GeoPoint(lat=lat, lon=lon), satellites=0 if self.fly else 8,
            hdop=None if self.fly else 1.1,
            heading_valid=True, heading_deg=heading, rssi_dbm=-55 - 5 * self.idx,
            target_count=len(confirmed), learning_background=False, background_cells=14,
            alarm_active=self.alarm)
        for pid, d, a, v, c, k, hit in ppl[:6]:
            m.tracks.add(id=pid, distance_m=d, angle_deg=a, speed_mps=v, confirmed=c,
                         detection_class=k if c else pb.DETECTION_CLASS_UNSPECIFIED, hit_ratio=hit)
        return m

    async def do_handshake(self, r, w):
        """Syn -> Ack, как TelemetryLink.h. False — сервер отказал (или нет Ack в режиме strict)."""
        if self.handshake == "off":
            print(f"[{self.name}] подключено, без рукопожатия")
            return True
        syn = sa.Syn(mac=self.mac)
        syn.onboards.add(name="camera", url=f"http://127.0.0.1:{8100 + self.idx}/stream")
        w.write(frame(syn.SerializeToString()))
        await w.drain()
        mac = ":".join(f"{b:02x}" for b in self.mac)
        try:
            wait = 10 if self.handshake == "strict" else 3
            n = int.from_bytes(await asyncio.wait_for(r.readexactly(4), wait), "big")
            ack = sa.Ack()
            ack.ParseFromString(await asyncio.wait_for(r.readexactly(n), wait))
        except asyncio.TimeoutError:
            if self.handshake == "strict":
                print(f"[{self.name}] нет Ack за 10 с")
                return False
            print(f"[{self.name}] сервер не ответил Ack — старый протокол, продолжаю")
            return True
        if ack.code:
            print(f"[{self.name}] сервер отказал: Ack code {ack.code}, повтор через 10 с")
            return False
        print(f"[{self.name}] подключено, Syn/Ack — принят, MAC {mac}")
        return True

    async def run(self):
        while True:
            try:
                r, w = await asyncio.open_connection(self.host, self.port, local_addr=self.local)
            except OSError as e:
                if self.local and "assign" in str(e).lower():
                    print(f"[{self.name}] нельзя занять адрес {self.local[0]}, подключаюсь без него")
                    self.local = None
                    continue
                print(f"[{self.name}] нет сервера ({e}), повтор через 2 с")
                await asyncio.sleep(2)
                continue
            try:
                if not await self.do_handshake(r, w):
                    w.close()
                    await asyncio.sleep(10)
                    continue
                print(f"[{self.name}] шлю снимки 5 раз в секунду")
                while True:
                    d = self.snapshot(time.time()).SerializeToString()
                    w.write(frame(d))
                    await w.drain()
                    await asyncio.sleep(0.2)
            except (ConnectionError, OSError, asyncio.IncompleteReadError):
                pass
            w.close()
            print(f"[{self.name}] связь потеряна, переподключаюсь")
            await asyncio.sleep(1)


async def main():
    ap = argparse.ArgumentParser(description="Имитатор устройств с радаром")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--count", type=int, default=1)
    ap.add_argument("--handshake", choices=["auto", "strict", "off"], default="auto",
                    help="Syn/Ack: auto — нет Ack за 3 с, продолжать (старый сервер); strict; off — старый протокол")
    ap.add_argument("--fly", action="store_true",
                    help="устройство летает по помещению (прямоугольник 4×3 м) — для трека на сервере")
    a = ap.parse_args()
    await asyncio.gather(*(SimDevice(i, a.host, a.port, a.handshake, a.fly).run() for i in range(a.count)))


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
