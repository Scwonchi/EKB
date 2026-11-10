"""Веб-камера на второй Raspberry Pi → кадры по HTTP для подъезда к окну.

Запускается на второй Raspberry Pi, к которой подключена USB-веб-камера. ROS там не нужен:
только Python 3 и OpenCV (``sudo apt install python3-opencv``). Файл можно просто скопировать:

    python3 webcam_server.py --device 0 --width 640 --height 480 --port 8080

Нода подъезда на ровере берёт кадры с ``http://<адрес второй Pi>:8080/frame.jpg``
(``frame_source: http``, ``http_url``).

    /frame.jpg  последний кадр (JPEG); заголовки X-Frame-Age-Ms — сколько мс назад снят,
                X-Frame-Seq — номер кадра. Часы двух Raspberry Pi сверять не нужно.
    /stream     видеопоток MJPEG — посмотреть в браузере, куда смотрит камера
    /           страница с потоком
    /health     JSON: камера работает, частота кадров, размер

Камера читается непрерывно в отдельном потоке, поэтому /frame.jpg всегда отдаёт свежий кадр,
а не застрявший в буфере драйвера.
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2


class Grabber:
    def __init__(self, device, width: int, height: int, fps: float, fourcc: str, rotate: int, quality: int):
        self.device, self.size, self.fps, self.fourcc = device, (width, height), fps, fourcc
        self.rotate, self.quality = rotate, quality
        self.lock = threading.Condition()
        self.frame = None
        self.stamp = 0.0
        self.seq = 0
        self._jpeg: Optional[bytes] = None
        self._jpeg_seq = -1
        self.rate = 0.0
        self.error = ""
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2) if isinstance(self.device, int) else cv2.VideoCapture(self.device)
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _loop(self):
        cap = None
        n, t0 = 0, time.monotonic()
        while self.running:
            if cap is None or not cap.isOpened():
                cap = self._open()
                if not cap.isOpened():
                    self.error = f"камера {self.device} не открывается"
                    time.sleep(1.0)
                    cap = None
                    continue
            ok, img = cap.read()
            if not ok or img is None:
                self.error = "камера не отдаёт кадры — переоткрываю"
                cap.release()
                cap = None
                time.sleep(0.5)
                continue
            if self.rotate == 180:
                img = cv2.rotate(img, cv2.ROTATE_180)
            elif self.rotate == 90:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotate == 270:
                img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            with self.lock:
                self.frame, self.stamp, self.error = img, time.monotonic(), ""
                self.seq += 1
                self.lock.notify_all()
            n += 1
            if time.monotonic() - t0 >= 2.0:
                self.rate, n, t0 = n / (time.monotonic() - t0), 0, time.monotonic()

    def jpeg(self):
        """(jpeg, возраст кадра в секундах, номер) или None."""
        with self.lock:
            if self.frame is None:
                return None
            if self._jpeg_seq != self.seq:
                ok, buf = cv2.imencode(".jpg", self.frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    return None
                self._jpeg, self._jpeg_seq = buf.tobytes(), self.seq
            return self._jpeg, time.monotonic() - self.stamp, self.seq

    def wait_next(self, seq: int, timeout: float = 1.0) -> None:
        with self.lock:
            self.lock.wait_for(lambda: self.seq != seq, timeout)


PAGE = b"""<!doctype html><meta charset="utf-8"><title>Webcam</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif}img{max-width:100%;display:block;margin:auto}</style>
<p style="text-align:center">Webcam: /frame.jpg, /stream, /health</p><img src="stream">"""


def make_handler(g: Grabber):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):          # без строки в журнале на каждый кадр
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/frame.jpg", "/frame"):
                r = g.jpeg()
                if r is None:
                    self.send_error(503, g.error or "нет кадров")
                    return
                data, age, seq = r
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Frame-Age-Ms", f"{age * 1000:.0f}")
                self.send_header("X-Frame-Seq", str(seq))
                self.end_headers()
                self.wfile.write(data)
            elif path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        g.wait_next(seq)
                        r = g.jpeg()
                        if r is None:
                            continue
                        data, _, seq = r
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            elif path == "/health":
                r = g.jpeg()
                body = json.dumps({"ok": r is not None and r[1] < 1.0, "fps": round(g.rate, 1),
                                   "age_ms": round(r[1] * 1000) if r else None,
                                   "size": list(g.frame.shape[1::-1]) if g.frame is not None else None,
                                   "error": g.error}, ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            else:
                self.send_error(404)
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="0", help="номер камеры (0 = /dev/video0) или путь/URL")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--fourcc", default="MJPG", help="формат с камеры (MJPG разгружает USB); пусто — как есть")
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270), help="камера стоит вверх ногами — 180")
    ap.add_argument("--quality", type=int, default=85, help="качество JPEG")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    a = ap.parse_args(argv)
    dev = int(a.device) if a.device.isdigit() else a.device
    g = Grabber(dev, a.width, a.height, a.fps, a.fourcc, a.rotate, a.quality)
    srv = ThreadingHTTPServer((a.host, a.port), make_handler(g))
    srv.daemon_threads = True
    print(f"веб-камера {a.device}: http://{a.host}:{a.port}/frame.jpg  /stream  /health", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        g.running = False
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
