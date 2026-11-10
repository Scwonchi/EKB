"""
Связь с ESP-шлюзом по USB. Протокол — как в esp/gateway/gw_link.h:

    A5 | тип | клиент | длина (2 байта, LE) | данные | CRC-8 (0x07) по байтам от «тип»

Каждый клиент шлюза (устройство с радаром) превращается в обычное подключение:
у него есть StreamReader и «писатель», поэтому остальной сервер работает
с ним так же, как с TCP.
"""
import asyncio
import threading
import time

import serial  # pyserial

SYNC = 0xA5
CONNECT, DATA, DISCONNECT, STATUS, RESET = 1, 2, 3, 4, 5
MAX_PAYLOAD = 1024


def crc8(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(ftype: int, client: int, data: bytes = b"") -> bytes:
    body = bytes([ftype, client, len(data) & 0xFF, len(data) >> 8]) + data
    return bytes([SYNC]) + body + bytes([crc8(body)])


class Parser:
    """Сборка кадров из потока байт. feed() возвращает список (тип, клиент, данные)."""

    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes([SYNC]))
            if i < 0:
                self.buf.clear()
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 5:
                return out
            length = self.buf[3] | (self.buf[4] << 8)
            if length > MAX_PAYLOAD:
                self.errors += 1
                del self.buf[:1]
                continue
            total = 5 + length + 1
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[1:5 + length])
            if crc8(body) != self.buf[5 + length]:
                self.errors += 1
                del self.buf[:1]              # ищем следующий A5
                continue
            out.append((body[0], body[1], body[4:]))
            del self.buf[:total]


class GatewayWriter:
    """Похож на asyncio.StreamWriter — этого достаточно для Device в server.py."""

    def __init__(self, gw, client, ip):
        self.gw, self.client, self.ip = gw, client, ip
        self.closed = False

    def write(self, data: bytes):
        if self.closed:
            return
        for k in range(0, len(data), MAX_PAYLOAD):
            self.gw.send(encode(DATA, self.client, data[k:k + MAX_PAYLOAD]))

    async def drain(self):
        pass

    def close(self):
        if not self.closed:
            self.closed = True
            self.gw.send(encode(DISCONNECT, self.client))

    def get_extra_info(self, name):
        return (self.ip, f"шлюз#{self.client}") if name == "peername" else None


class SerialGateway:
    def __init__(self, port, baud, on_client, log):
        self.port, self.baud = port, baud
        self.on_client = on_client          # async (reader, writer) — как у asyncio.start_server
        self.log = log
        self.ser = None
        self.lock = threading.Lock()
        self.readers = {}
        self.parser = Parser()
        self.last_status = None
        self.last_status_log = 0.0

    def send(self, frame: bytes):
        with self.lock:
            if self.ser:
                try:
                    self.ser.write(frame)
                except serial.SerialException:
                    pass                         # поток чтения заметит обрыв

    def _open(self):
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = self.port, self.baud, 0.05
        s.dtr = False                            # иначе плата с USB-UART мостом перезагрузится
        s.rts = False
        s.open()
        return s

    async def run(self):
        loop = asyncio.get_running_loop()
        while True:
            try:
                ser = await loop.run_in_executor(None, self._open)
            except serial.SerialException as e:
                self.log(f"шлюз: не открыть {self.port} ({e}); повтор через 3 с")
                await asyncio.sleep(3)
                continue
            with self.lock:
                self.ser = ser
            self.log(f"шлюз: порт {self.port} открыт")
            self.send(encode(RESET, 0))         # устройства переподключатся и продолжат слать снимки
            stop = threading.Event()
            done = loop.create_future()
            threading.Thread(target=self._reader, args=(ser, loop, stop, done), daemon=True).start()
            await done
            stop.set()
            with self.lock:
                self.ser = None
            try:
                ser.close()
            except Exception:
                pass
            for r in self.readers.values():
                r.feed_eof()
            self.readers.clear()
            self.log("шлюз: порт закрыт (шлюз отключён?); повтор через 3 с")
            await asyncio.sleep(3)

    def _reader(self, ser, loop, stop, done):
        try:
            while not stop.is_set():
                data = ser.read(4096)
                if data:
                    frames = self.parser.feed(data)
                    if frames:
                        loop.call_soon_threadsafe(self._dispatch, frames)
        except (serial.SerialException, OSError):
            pass
        finally:
            loop.call_soon_threadsafe(lambda: done.done() or done.set_result(None))

    def _dispatch(self, frames):
        for ftype, client, data in frames:
            if ftype == CONNECT:
                old = self.readers.pop(client, None)
                if old:
                    old.feed_eof()
                ip = ".".join(str(b) for b in data[:4]) if len(data) >= 4 else "?"
                reader = asyncio.StreamReader()
                self.readers[client] = reader
                asyncio.ensure_future(self.on_client(reader, GatewayWriter(self, client, ip)))
            elif ftype == DATA:
                r = self.readers.get(client)
                if r:
                    r.feed_data(data)
            elif ftype == DISCONNECT:
                r = self.readers.pop(client, None)
                if r:
                    r.feed_eof()
            elif ftype == STATUS and len(data) >= 5:
                n, up = data[0], int.from_bytes(data[1:5], "little")
                wifi = data[5] if len(data) >= 6 else None
                changed = self.last_status is None or self.last_status[0] != n or self.last_status[3] != wifi
                self.last_status = (n, up, time.time(), wifi)
                if changed or time.time() - self.last_status_log > 60:
                    self.last_status_log = time.time()
                    w = f"в Wi-Fi шлюза {wifi}, " if wifi is not None else ""
                    self.log(f"шлюз на связи: {w}подключено к серверу {n}, работает {up} с")
                    if wifi and n < wifi:
                        self.log("  есть устройства в Wi-Fi без подключения к серверу: проверьте SERVER_HOST = 192.168.4.1 и порт 9000")
