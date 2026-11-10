"""
Проверка сервера на соответствие api/openapi.yaml — для тех, кто пишет свой сервер.

    pip install pyyaml jsonschema
    python tools/sim_device.py --count 2 &            # нужен хотя бы один источник данных
    python tools/check_api.py http://localhost:8080

Проверяет все методы /api/v1, коды и тела ошибок, заголовки CORS и поток событий
(для потока подождите 10–15 с после запуска имитатора, чтобы были тревоги):
каждый ответ и каждое событие сверяются со схемами из openapi.yaml.
Код выхода 0 — всё совпало, 1 — есть расхождения.
"""
import argparse
import datetime as dt
import http.client
import json
import os
import sys
import time
import urllib.parse

import yaml
from jsonschema import Draft202012Validator

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = yaml.safe_load(open(os.path.join(HERE, "..", "api", "openapi.yaml"), encoding="utf-8"))
SSE_SCHEMAS = {k: v["$ref"].split("/")[-1] for k, v in SPEC["paths"]["/api/v1/events"]["get"]["x-sse-events"].items()}

ok = fail = 0


def report(good, what, detail=""):
    global ok, fail
    if good:
        ok += 1
        print(f"  ок    {what}")
    else:
        fail += 1
        print(f"  ОШИБКА {what}" + (f"\n         {detail}" if detail else ""))


def timestamps(obj, path=""):
    """Все строки, похожие на время, должны разбираться как ISO 8601."""
    bad = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k.endswith("_at") or k in ("server_time", "received_at"):
                if v is not None:
                    try:
                        dt.datetime.fromisoformat(str(v).replace("Z", "+00:00"))
                    except ValueError:
                        bad.append(f"{path}.{k}={v!r}")
            bad += timestamps(v, f"{path}.{k}")
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            bad += timestamps(v, f"{path}[{i}]")
    return bad


def check_schema(obj, schema_name, what):
    v = Draft202012Validator({"$ref": f"#/components/schemas/{schema_name}", "components": SPEC["components"]})
    errors = sorted(v.iter_errors(obj), key=lambda e: list(e.path))
    errors_t = timestamps(obj)
    detail = "; ".join(f"{'/'.join(map(str, e.path)) or '(корень)'}: {e.message}" for e in errors[:3])
    if errors_t:
        detail += ("; " if detail else "") + "неверное время: " + ", ".join(errors_t[:3])
    report(not errors and not errors_t, f"{what} соответствует схеме {schema_name}", detail)


def request(base, method, path, body=None, headers=None):
    u = urllib.parse.urlsplit(base)
    c = http.client.HTTPConnection(u.hostname, u.port or 80, timeout=10)
    h = dict(headers or {})
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        h.setdefault("Content-Type", "application/json")
    c.request(method, path, body=data, headers=h)
    r = c.getresponse()
    raw = r.read()
    hdrs = {k.lower(): v for k, v in r.getheaders()}
    c.close()
    try:
        js = json.loads(raw) if raw else None
    except ValueError:
        js = None
    return r.status, hdrs, js


def expect_error(base, method, path, status, code, what, body=None, headers=None):
    st, h, js = request(base, method, path, body, headers)
    report(st == status, f"{what}: код {status}", f"получен {st}")
    if js is not None:
        check_schema(js, "Error", what)
        report(js.get("error", {}).get("code") == code, f"{what}: error.code = {code}", f"получен {js}")
    else:
        report(False, f"{what}: тело ошибки — JSON")


def read_events(base, seconds):
    u = urllib.parse.urlsplit(base)
    c = http.client.HTTPConnection(u.hostname, u.port or 80, timeout=seconds + 5)
    c.request("GET", "/api/v1/events", headers={"Accept": "text/event-stream"})
    r = c.getresponse()
    ctype = r.getheader("Content-Type", "")
    events, name, data = [], None, []
    end = time.time() + seconds
    while time.time() < end:
        line = r.fp.readline()
        if not line:
            break
        line = line.decode().rstrip("\r\n")
        if line == "":
            if data:
                events.append((name or "message", "\n".join(data)))
            name, data = None, []
        elif line.startswith("event:"):
            name = line[6:].strip()
        elif line.startswith("data:"):
            data.append(line[5:].lstrip())
    c.close()
    return r.status, ctype, r.getheader("Access-Control-Allow-Origin"), events


def main():
    ap = argparse.ArgumentParser(description="Проверка сервера по api/openapi.yaml")
    ap.add_argument("base", nargs="?", default="http://localhost:8080")
    ap.add_argument("--events-seconds", type=float, default=4)
    a = ap.parse_args()
    base = a.base.rstrip("/")

    print("Сервер и CORS")
    st, h, js = request(base, "GET", "/api/v1/info")
    report(st == 200, "GET /api/v1/info: 200", f"получен {st}")
    check_schema(js, "Info", "info")
    report(h.get("access-control-allow-origin") == "*", "CORS: Access-Control-Allow-Origin: *", str(h.get("access-control-allow-origin")))
    report("application/json" in h.get("content-type", ""), "Content-Type: application/json", h.get("content-type"))
    st, h, _ = request(base, "OPTIONS", "/api/v1/devices",
                       headers={"Origin": "http://example", "Access-Control-Request-Method": "GET"})
    report(st in (200, 204) and "GET" in h.get("access-control-allow-methods", ""),
           "CORS: предварительный запрос OPTIONS", f"{st} {h}")

    print("Устройства")
    st, _, js = request(base, "GET", "/api/v1/devices")
    report(st == 200, "GET /api/v1/devices: 200", f"получен {st}")
    report(isinstance(js, dict) and isinstance(js.get("devices"), list), "ответ {devices: [...]}")
    devs = js.get("devices", []) if isinstance(js, dict) else []
    for d in devs:
        check_schema(d, "Device", f"устройство {d.get('id')}")
    online = [d for d in devs if d.get("online")]
    if not online:
        print("  (нет подключённых устройств — запустите tools/sim_device.py; часть проверок пропущена)")
    else:
        dev = online[0]["id"]
        st, _, js = request(base, "GET", f"/api/v1/devices/{urllib.parse.quote(dev)}")
        report(st == 200, f"GET /api/v1/devices/{dev}: 200", f"получен {st}")
        check_schema(js, "Device", "одно устройство")
        st, _, js = request(base, "GET", f"/api/v1/devices/{urllib.parse.quote(dev)}/tracks")
        if st == 200:
            report(True, f"GET /api/v1/devices/{dev}/tracks: 200")
            check_schema(js, "Tracks", "цели")
        else:
            expect_error(base, "GET", f"/api/v1/devices/{urllib.parse.quote(dev)}/tracks", 404, "no_data", "снимков ещё нет")
        st, _, js = request(base, "GET", f"/api/v1/devices/{urllib.parse.quote(dev)}/path")
        report(st == 200, f"GET /api/v1/devices/{dev}/path: 200", f"получен {st}")
        check_schema(js, "Path", "трек")
    expect_error(base, "GET", "/api/v1/devices/no-such-device", 404, "device_not_found", "неизвестное устройство")
    expect_error(base, "GET", "/api/v1/devices/no-such-device/tracks", 404, "device_not_found", "цели неизвестного устройства")

    print("Тревоги")
    for q in ("", "?active=true", "?active=false&limit=5"):
        st, _, js = request(base, "GET", f"/api/v1/alarms{q}")
        report(st == 200, f"GET /api/v1/alarms{q}: 200", f"получен {st}")
        for al in (js or {}).get("alarms", [])[:20]:
            check_schema(al, "Alarm", f"тревога {al.get('id')}")
        if q.startswith("?active=true"):
            report(all(al.get("active") for al in (js or {}).get("alarms", [])), "фильтр active=true")
    expect_error(base, "GET", "/api/v1/alarms?limit=abc", 400, "bad_request", "неверный limit")
    expect_error(base, "GET", "/api/v1/nothing", 404, "not_found", "неизвестный путь")

    print(f"Поток событий ({a.events_seconds:.0f} с)")
    st, ctype, acao, events = read_events(base, a.events_seconds)
    report(st == 200 and "text/event-stream" in ctype, "GET /api/v1/events: 200, text/event-stream", f"{st} {ctype}")
    report(acao == "*", "CORS у потока событий", str(acao))
    report(bool(events) and events[0][0] == "snapshot", "первое событие — snapshot",
           f"первое: {events[0][0] if events else 'нет событий'}")
    seen = {}
    for name, data in events:
        seen[name] = seen.get(name, 0) + 1
        if name not in SSE_SCHEMAS:
            report(False, f"событие {name!r} описано в x-sse-events")
            continue
        if seen[name] <= 3:                                      # первые три каждого вида
            try:
                check_schema(json.loads(data), SSE_SCHEMAS[name], f"событие {name}")
            except ValueError:
                report(False, f"событие {name}: данные — JSON", data[:80])
    print("  получено событий: " + ", ".join(f"{k} {v}" for k, v in seen.items()))

    print(f"\nИтого: совпало {ok}, расхождений {fail}")
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
