"""Связь микродрона с ровером: строки JSON по TCP, микродрон подключается сам.

Адрес ровера в сети площадки заранее неизвестен (его раздаёт роутер), поэтому ровер раз в
секунду рассылает UDP-маяк {"ekb": "rover", "port": 8770}, а микродрон слушает маяк и
подключается к отправителю. Можно задать адрес и явно (rover_host).

Сообщения — по одному JSON-объекту в строке, поле "t" — тип:

  микродрон -> ровер   hello, state, event, detection, victims, done, plan_ok, plan_bad, pong, ping
  ровер -> микродрон   plan, go, abort, ping, pong

Важные сообщения (тревоги, итог) при обрыве не теряются: копятся в очереди и уходят после
переподключения. state при обрыве отбрасывается.
"""
import json
import socket
import threading

try:
    import queue
except ImportError:                                   # pragma: no cover
    import Queue as queue

PORT = 8770
BEACON_PORT = 8771


class Link(object):
    required = True

    def __init__(self, clock, host=None, port=PORT, beacon_port=BEACON_PORT, log=None):
        self.clock, self.host, self.port, self.beacon_port = clock, host, port, beacon_port
        self.log = log or (lambda s: None)
        self.rx = queue.Queue()
        self.keep = []                                 # неотправленные важные сообщения
        self.lock = threading.Lock()
        self.sock = None
        self.last_rx = clock.time()
        self.peer = None
        self.running = False
        self._ping_t = 0.0

    # ---------- для программы ----------
    def start(self):
        self.running = True
        t = threading.Thread(target=self._loop, name="rover-link")
        t.daemon = True
        t.start()

    def stop(self):
        self.running = False
        self._close()

    def connected(self):
        return self.sock is not None

    def age(self):
        """Сколько секунд от ровера ничего не приходило."""
        return self.clock.time() - self.last_rx

    def poll(self):
        """Принятые сообщения (кроме ping/pong). Заодно раз в 0,5 с шлёт ping."""
        now = self.clock.time()
        if now - self._ping_t >= 0.5:
            self._ping_t = now
            self.send({"t": "ping"}, keep=False)
        out = []
        while True:
            try:
                out.append(self.rx.get_nowait())
            except queue.Empty:
                return out

    def send(self, msg, keep=True):
        data = (json.dumps(msg, ensure_ascii=False) + "\n").encode("utf-8")
        with self.lock:
            s = self.sock
            if s is not None:
                try:
                    s.sendall(data)
                    return True
                except (OSError, socket.error):
                    self._drop()
            if keep:
                self.keep.append(data)
                del self.keep[:-200]
        return False

    # ---------- поток связи ----------
    def _drop(self):
        s, self.sock = self.sock, None
        if s is not None:
            try:
                s.close()
            except (OSError, socket.error):
                pass

    def _close(self):
        with self.lock:
            self._drop()

    def _find_rover(self):
        if self.host:
            return self.host, self.port
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            u.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            u.bind(("", self.beacon_port))
            u.settimeout(1.0)
            while self.running:
                try:
                    data, addr = u.recvfrom(512)
                except socket.timeout:
                    continue
                try:
                    m = json.loads(data.decode("utf-8"))
                except ValueError:
                    continue
                if m.get("ekb") == "rover":
                    return addr[0], int(m.get("port", self.port))
        finally:
            u.close()
        return None

    def _loop(self):
        import time
        while self.running:
            try:
                target = self._find_rover()
                if target is None:
                    continue
                s = socket.create_connection(target, timeout=3.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(1.0)
            except (OSError, socket.error) as e:
                self.log("ровер недоступен: %s" % (e,))
                time.sleep(1.0)
                continue
            with self.lock:
                self.sock, self.peer = s, target
                pending, self.keep = self.keep, []
                try:
                    for data in pending:
                        s.sendall(data)
                except (OSError, socket.error):
                    self.keep = pending + self.keep
                    self._drop()
                    continue
            self.last_rx = self.clock.time()
            self.log("связь с ровером %s:%d" % target)
            self.rx.put({"t": "_connected"})
            buf = b""
            while self.running and self.sock is s:
                try:
                    chunk = s.recv(4096)
                except socket.timeout:
                    continue
                except (OSError, socket.error):
                    break
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    if not line.strip():
                        continue
                    try:
                        m = json.loads(line.decode("utf-8"))
                    except ValueError:
                        continue
                    self.last_rx = self.clock.time()
                    t = m.get("t")
                    if t == "ping":
                        self.send({"t": "pong"}, keep=False)
                    elif t != "pong":
                        self.rx.put(m)
            self._close()
            self.log("связь с ровером потеряна")
