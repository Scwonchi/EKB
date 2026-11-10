"""
Страница трека и радара для сервера iot-go — без изменений в самом iot-go.

iot-go отдаёт веб-клиентам список устройств (GET /api/v1/devices) и поток снимков каждого
устройства по WebSocket (/api/v1/devices/{id}), но WebSocket принимает только страницы со своего
же адреса (same-origin). Эта программа отдаёт страницу index.html и пробрасывает /api/... в iot-go —
для браузера страница и API на одном адресе, iot-go её принимает.

    python iotgo_view.py                          # iot-go на этом же ПК, порт 8080
    python iotgo_view.py --iotgo 192.168.1.40:8080 --port 8090
    открыть http://localhost:8090/

Трек собирает страница из снимков (position, heading_deg): iot-go истории не хранит.
Нужен только Python 3.8+, без библиотек.
"""
import argparse
import asyncio
import os

HERE = os.path.dirname(os.path.abspath(__file__))


async def pipe(r, w):
    try:
        while True:
            d = await r.read(65536)
            if not d:
                break
            w.write(d)
            await w.drain()
    except (ConnectionError, OSError):
        pass
    finally:
        try:
            w.close()
        except Exception:
            pass


async def handle(reader, writer, upstream):
    try:
        head = await reader.readuntil(b"\r\n\r\n")
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError, ConnectionError):
        writer.close()
        return
    line = head.split(b"\r\n", 1)[0].decode("latin-1")
    parts = line.split(" ")
    path = parts[1] if len(parts) > 1 else "/"
    if not path.startswith("/api/"):
        if path.split("?")[0] in ("/", "/index.html"):
            body = open(os.path.join(HERE, "index.html"), "rb").read()
            writer.write(b"HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-store\r\n"
                         b"Content-Length: %d\r\nConnection: close\r\n\r\n" % len(body) + body)
        else:
            writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
        await writer.drain()
        writer.close()
        return
    host, port = upstream
    try:
        ur, uw = await asyncio.open_connection(host, port)
    except OSError as e:
        msg = f'{{"error":"iot-go {host}:{port} не отвечает: {e}"}}'.encode()
        writer.write(b"HTTP/1.1 502 Bad Gateway\r\nContent-Type: application/json\r\nContent-Length: %d\r\n"
                     b"Connection: close\r\n\r\n" % len(msg) + msg)
        await writer.drain()
        writer.close()
        return
    if b"\r\nupgrade: websocket" not in head.lower():
        # Обычный запрос: одно соединение — один запрос, чтобы браузер не повёл по нему запрос страницы
        lines = [l for l in head.split(b"\r\n")[:-2] if not l.lower().startswith(b"connection:")]
        head = b"\r\n".join(lines) + b"\r\nConnection: close\r\n\r\n"
    # Host и Origin не меняем: для iot-go страница и запрос с одного адреса — WebSocket принимается
    uw.write(head)
    await uw.drain()
    await asyncio.gather(pipe(reader, uw), pipe(ur, writer))


async def main():
    ap = argparse.ArgumentParser(description="Страница трека для iot-go")
    ap.add_argument("--iotgo", default="127.0.0.1:8080", help="адрес HTTP API iot-go (server_http), по умолчанию 127.0.0.1:8080")
    ap.add_argument("--port", type=int, default=8090, help="порт страницы, по умолчанию 8090")
    ap.add_argument("--host", default="0.0.0.0")
    a = ap.parse_args()
    h, p = a.iotgo.rsplit(":", 1)
    srv = await asyncio.start_server(lambda r, w: handle(r, w, (h, int(p))), a.host, a.port)
    print(f"Страница: http://localhost:{a.port}/  ->  iot-go http://{a.iotgo}/api/v1", flush=True)
    await srv.serve_forever()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
