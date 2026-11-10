"""
HTTP API v1 и страница оператора. Описание API — api/openapi.yaml, руководство — docs/web-api.md.

Только стандартная библиотека Python: маленький HTTP-сервер на asyncio.

  GET  /                                  страница оператора (web/index.html) — пример клиента API
  GET  /api/v1/info                       версия API, время сервера
  GET  /api/v1/devices                    устройства с последним состоянием
  GET  /api/v1/devices/{id}               одно устройство
  GET  /api/v1/devices/{id}/tracks        последние цели радара
  GET  /api/v1/alarms[?active=true&limit=N]
  GET  /api/v1/events                     поток событий (Server-Sent Events)

Команд устройствам нет: протокол устройств односторонний (устройство только шлёт снимки).

Все ответы /api — JSON, с заголовками CORS: страницу может держать любой другой веб-сервер.
"""
import asyncio
import collections
import datetime as dt
import json
import math
import os
import time
from urllib.parse import parse_qs, unquote, urlsplit

from telemetry.v1 import telemetry_pb2 as pb

API_VERSION = "1.0"
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

CLASS = {pb.DETECTION_CLASS_UNSPECIFIED: "unspecified", pb.DETECTION_CLASS_EMPTY: "empty",
         pb.DETECTION_CLASS_MOVING: "moving", pb.DETECTION_CLASS_STATIONARY: "stationary",
         pb.DETECTION_CLASS_INTERFERENCE: "interference"}
STATUS_EVENT_S = 1.0          # событие status — не чаще раза в секунду на устройство
PATH_STEP_M = 0.2             # трек: новая точка через столько метров (или поворот на 15°)
PATH_MAX = 5000               # точек трека на устройство


def now_iso():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def r1(x, nd=1):
    return round(float(x), nd)


def geo(p):
    return {"lat": round(p.lat, 7), "lon": round(p.lon, 7)}


def wrap360(a):
    return round(a % 360.0, 1)


class ApiError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code, self.message = status, code, message


class WebHub:
    """Состояние для API (в виде готовых JSON-объектов) и рассылка событий."""

    def __init__(self, log):
        self.log = log
        self.clients = set()
        self.devices = collections.OrderedDict()   # id -> Device
        self.tracks = {}                           # id -> Tracks
        self.alarms = collections.OrderedDict()    # alarm.id -> Alarm (новые в конце)
        self.status_sent = {}                      # id -> время последнего события status
        self.paths = {}                            # id -> трек: точки position (схема PathPoint)
        self.event_id = 0

    # ---------- вызовы из server.py ----------
    def device_online(self, device_id, address, mac=None, onboards=()):
        d = self._device(device_id)
        d.update(online=True, address=address, connected_at=now_iso(), mac=mac, onboards=list(onboards))
        self._emit("device", d)

    def device_offline(self, device_id):
        if device_id in self.devices:
            d = self.devices[device_id]
            d["online"] = False
            self._emit("device", d)

    def telemetry(self, device_id, t):
        """Снимок telemetry.v1.Telemetry: обновляет состояние и цели устройства."""
        d = self._device(device_id)
        if d["firmware_version"] != t.firmware_version:
            d["firmware_version"] = t.firmware_version
            self._emit("device", d)
        heading = t.heading_deg if t.heading_valid and t.HasField("heading_deg") else None
        d["status"] = {
            "received_at": now_iso(),
            "uptime_s": t.uptime_s,
            "radar_ok": t.radar_ok,
            "people": t.target_count,
            "detection_class": CLASS.get(t.detection_class, "unspecified"),
            "person_probability": r1(t.person_probability, 2),
            "position": geo(t.position) if t.HasField("position") else None,
            "satellites": t.satellites,
            "hdop": r1(t.hdop) if t.HasField("hdop") else None,
            "heading_deg": r1(heading) if heading is not None else None,
            "wifi_rssi_dbm": t.rssi_dbm,
            "alarm_active": t.alarm_active,
        }
        if t.HasField("position"):
            self._path_point(device_id, t.position.lat, t.position.lon, heading)
        loop_t = time.monotonic()
        if loop_t - self.status_sent.get(device_id, 0) >= STATUS_EVENT_S:
            self.status_sent[device_id] = loop_t
            self._emit("status", {"device_id": device_id, "status": d["status"]})

        obj = {
            "device_id": device_id,
            "received_at": d["status"]["received_at"],
            "learning_background": t.learning_background,
            "background_cells": t.background_cells,
            "targets": [{
                "id": x.id,
                "distance_m": r1(x.distance_m, 2),
                "angle_deg": r1(x.angle_deg),
                "bearing_deg": wrap360(heading + x.angle_deg) if heading is not None else None,
                "speed_mps": r1(x.speed_mps, 2),
                "confirmed": x.confirmed,
                "detection_class": CLASS.get(x.detection_class, "unspecified") if x.confirmed else "unspecified",
                "hit_ratio": r1(x.hit_ratio, 2),
            } for x in t.tracks],
        }
        self.tracks[device_id] = obj
        self._emit("tracks", obj)

    def _path_point(self, device_id, lat, lon, heading):
        """Трек устройства: новая точка, если сдвинулось на PATH_STEP_M или повернулось на 15°."""
        pts = self.paths.setdefault(device_id, [])
        if pts:
            p = pts[-1]
            dn = (lat - p["lat"]) * 111195.0
            de = (lon - p["lon"]) * 111195.0 * math.cos(math.radians(lat))
            turn = (abs((heading - p["heading_deg"] + 180) % 360 - 180)
                    if heading is not None and p["heading_deg"] is not None else 0)
            if math.hypot(dn, de) < PATH_STEP_M and turn < 15:
                return
        pt = {"t": now_iso(), "lat": round(lat, 7), "lon": round(lon, 7),
              "heading_deg": r1(heading) if heading is not None else None}
        pts.append(pt)
        if len(pts) > PATH_MAX:
            del pts[:len(pts) - PATH_MAX]
        self._emit("path", {"device_id": device_id, "point": pt})

    def alarm(self, a):
        """Новая или изменившаяся тревога (схема Alarm; создаёт server.py)."""
        self.alarms[a["id"]] = a
        self.alarms.move_to_end(a["id"])
        while len(self.alarms) > 500:
            self.alarms.popitem(last=False)
        d = self._device(a["device_id"])
        if a["active"]:
            d["active_alarm_id"] = a["id"]
        elif d.get("active_alarm_id") == a["id"]:
            d["active_alarm_id"] = None
        self._emit("alarm", a)

    # ---------- данные ----------
    def _device(self, device_id):
        if device_id not in self.devices:
            self.devices[device_id] = {"id": device_id, "online": False, "address": None, "firmware_version": None,
                                       "connected_at": None, "status": None, "active_alarm_id": None,
                                       "mac": None, "onboards": []}
        return self.devices[device_id]

    def _alarm_list(self, active=None, limit=100):
        out = [a for a in reversed(self.alarms.values()) if active is None or a["active"] == active]
        return out[:limit]

    def snapshot(self):
        return {"api_version": API_VERSION, "server_time": now_iso(),
                "devices": list(self.devices.values()),
                "tracks": list(self.tracks.values()),
                "alarms": self._alarm_list(limit=100)}

    def _emit(self, event, data):
        self.event_id += 1
        payload = f"event: {event}\nid: {self.event_id}\ndata: {json.dumps(data, ensure_ascii=False)}\n\n".encode()
        for q in list(self.clients):
            if q.qsize() < 300:               # медленный клиент пропускает события, сервер его не ждёт
                q.put_nowait(payload)

    # ---------- HTTP ----------
    async def handle(self, reader, writer):
        try:
            head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 10)
            lines = head.decode("latin-1").split("\r\n")
            method, target, _ = lines[0].split(" ", 2)
            headers = {k.strip().lower(): v.strip() for k, v in (l.split(":", 1) for l in lines[1:] if ":" in l)}
            url = urlsplit(target)
            path, query = unquote(url.path).rstrip("/") or "/", parse_qs(url.query)
            body = b""
            n = int(headers.get("content-length", "0") or 0)
            if n:
                if n > 65536:
                    raise ApiError(413, "bad_request", "Слишком большой запрос")
                body = await reader.readexactly(n)

            if method == "OPTIONS" and path.startswith("/api/"):
                await self._send(writer, 204, b"", "text/plain")
            elif method == "GET" and path in ("/", "/index.html"):
                with open(os.path.join(WEB_DIR, "index.html"), "rb") as f:
                    await self._send(writer, 200, f.read(), "text/html; charset=utf-8", cors=False)
            elif path == "/api/v1/events" and method == "GET":
                await self._events(writer)
            elif path.startswith("/api/"):
                status, obj = await self._api(method, path, query, headers, body)
                await self._json(writer, status, obj)
            else:
                await self._send(writer, 404, "Нет такой страницы".encode(), "text/plain; charset=utf-8", cors=False)
        except ApiError as e:
            await self._json(writer, e.status, {"error": {"code": e.code, "message": e.message}})
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError, ValueError, IndexError):
            pass
        finally:
            try:
                writer.close()
            except Exception:
                pass

    async def _api(self, method, path, query, headers, body):
        parts = path.split("/")[1:]                 # ["api", "v1", ...]
        if parts[:2] != ["api", "v1"]:
            raise ApiError(404, "not_found", "Неизвестная версия API, используйте /api/v1")
        rest = parts[2:]
        if rest == ["info"] and method == "GET":
            return 200, {"api_version": API_VERSION, "server_time": now_iso()}
        if rest == ["devices"] and method == "GET":
            return 200, {"devices": list(self.devices.values())}
        if rest == ["alarms"] and method == "GET":
            active = query.get("active", [None])[0]
            active = None if active is None else active.lower() in ("1", "true", "yes")
            try:
                limit = max(1, min(500, int(query.get("limit", ["100"])[0])))
            except ValueError:
                raise ApiError(400, "bad_request", "limit должен быть числом от 1 до 500")
            return 200, {"alarms": self._alarm_list(active, limit)}
        if len(rest) >= 2 and rest[0] == "devices":
            dev_id = rest[1]
            if dev_id not in self.devices:
                raise ApiError(404, "device_not_found", f"Устройство {dev_id} не известно серверу")
            if len(rest) == 2 and method == "GET":
                return 200, self.devices[dev_id]
            if rest[2:] == ["path"] and method == "GET":
                return 200, {"device_id": dev_id, "points": self.paths.get(dev_id, [])}
            if rest[2:] == ["tracks"] and method == "GET":
                if dev_id not in self.tracks:
                    raise ApiError(404, "no_data", f"От устройства {dev_id} ещё не было снимков")
                return 200, self.tracks[dev_id]
        raise ApiError(404, "not_found", f"Нет такого метода: {method} {path}")

    async def _events(self, writer):
        writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream; charset=utf-8\r\n"
                     b"Cache-Control: no-store\r\nConnection: keep-alive\r\nAccess-Control-Allow-Origin: *\r\n\r\n"
                     b"retry: 3000\n\n")
        q = asyncio.Queue()
        self.event_id += 1
        q.put_nowait(f"event: snapshot\nid: {self.event_id}\ndata: "
                     f"{json.dumps(self.snapshot(), ensure_ascii=False)}\n\n".encode())
        self.clients.add(q)
        try:
            while True:
                try:
                    data = await asyncio.wait_for(q.get(), 15)
                except asyncio.TimeoutError:
                    data = b": keep-alive\n\n"        # чтобы прокси и браузер не закрыли соединение
                writer.write(data)
                await writer.drain()
        finally:
            self.clients.discard(q)

    async def _json(self, writer, status, obj):
        await self._send(writer, status, json.dumps(obj, ensure_ascii=False).encode(), "application/json; charset=utf-8")

    async def _send(self, writer, status, body, ctype, cors=True):
        reason = {200: "OK", 202: "Accepted", 204: "No Content", 400: "Bad Request", 404: "Not Found",
                  409: "Conflict", 413: "Payload Too Large", 415: "Unsupported Media Type"}.get(status, "OK")
        extra = ("Access-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, OPTIONS\r\n"
                 "Access-Control-Allow-Headers: Content-Type\r\n") if cors else ""
        writer.write(f"HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n"
                     f"Cache-Control: no-store\r\n{extra}Connection: close\r\n\r\n".encode() + body)
        await writer.drain()

    async def serve(self, host, port):
        srv = await asyncio.start_server(self.handle, host, port)
        self.log(f"страница оператора: http://localhost:{port}, API: http://localhost:{port}/api/v1")
        await srv.serve_forever()
