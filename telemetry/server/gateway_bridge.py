"""
Мост к любому TCP-серверу телеметрии (например, iot-go) с журналом миссии.

Два входа, можно оба сразу:
  --serial  ESP-шлюз по USB:
    ESP с радаром --Wi-Fi--> ESP-шлюз --USB--> gateway_bridge.py --TCP--> iot-go :9000
  --listen  TCP-порт — для ретранслятора на носителе, подключённого к Wi-Fi роутеру:
    разведчик --Wi-Fi--> ретранслятор (Mavic) --Wi-Fi роутера--> gateway_bridge.py :9001 --> iot-go :9000

Каждое устройство, подключившееся к шлюзу, получает своё TCP-подключение к серверу;
байты пересылаются как есть, в обе стороны, без разбора protobuf. Поэтому мост
не зависит от формата кадров и схемы. Отключилось устройство — мост закрывает его
подключение к серверу; сервер закрыл подключение — мост отключает устройство у шлюза,
и оно переподключается само.

    pip install pyserial protobuf
    python server/gateway_bridge.py --serial /dev/ttyACM0 --server 127.0.0.1:9000
    python server/gateway_bridge.py --listen 9001 --server 127.0.0.1:9000

Сервер iot-go увидит все устройства с адреса компьютера, на котором запущен мост
(у iot-go это не важно: он выдаёт UUID на каждое подключение).

Попутно мост ведёт журнал миссии (mission_log.py): копию байт от устройства разбирает
на снимки и пишет в missions/<дата>_<устройство>/ события с отметками времени, снимки,
карту обнаружения и report.html. На пересылку это не влияет. --log-dir "" — без журнала.
Для журнала нужен пакет protobuf (pip install protobuf) и server/gen (easyp generate).
"""
import argparse
import asyncio
import datetime as dt
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_log import MissionRecorder  # noqa: E402


def log(msg):
    print(f"[{dt.datetime.now():%H:%M:%S}] {msg}", flush=True)


class Bridge:
    def __init__(self, host, port, recorder=None):
        self.host, self.port = host, port
        self.active = 0
        self.recorder = recorder

    async def on_client(self, gw_reader, gw_writer):
        ip, via = gw_writer.get_extra_info("peername")
        name = f"{ip} ({via})"
        try:
            up_reader, up_writer = await asyncio.wait_for(asyncio.open_connection(self.host, self.port), 3)
        except (OSError, asyncio.TimeoutError) as e:
            log(f"{name}: сервер {self.host}:{self.port} недоступен ({e or 'нет ответа'}), отключаю устройство")
            gw_writer.close()
            return
        self.active += 1
        log(f"{name}: подключено к серверу {self.host}:{self.port} (устройств через мост: {self.active})")
        sent = 0
        tap = self.recorder.open(ip) if self.recorder else None     # журнал — по адресу устройства

        async def to_server():
            nonlocal sent
            while True:
                data = await gw_reader.read(4096)
                if not data:
                    return "устройство отключилось"
                up_writer.write(data)
                if tap:
                    try:
                        tap.feed(data, time.time())
                    except Exception as e:          # журнал не должен мешать пересылке
                        log(f"{name}: ошибка журнала {e!r}")
                await up_writer.drain()
                sent += len(data)

        async def to_device():
            while True:
                data = await up_reader.read(4096)
                if not data:
                    return "сервер закрыл подключение"
                gw_writer.write(data)          # iot-go ничего не шлёт, но мост прозрачный

        tasks = [asyncio.ensure_future(to_server()), asyncio.ensure_future(to_device())]
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()
        reason = next(iter(done)).exception() or next(iter(done)).result()
        up_writer.close()
        gw_writer.close()
        if tap:
            tap.close()
        self.active -= 1
        log(f"{name}: {reason}; передано {sent} байт (устройств через мост: {self.active})")


async def main():
    ap = argparse.ArgumentParser(description="Мост ESP-шлюз (USB) или ретранслятор (TCP) -> TCP-сервер телеметрии")
    ap.add_argument("--serial", help="порт шлюза, например /dev/ttyACM0 или COM5")
    ap.add_argument("--listen", type=int, help="TCP-порт для ретранслятора, например 9001")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--server", default="127.0.0.1:9000", help="адрес сервера, по умолчанию 127.0.0.1:9000")
    ap.add_argument("--log-dir", default="missions", help='папка журналов миссий ("" — без журнала)')
    a = ap.parse_args()
    if not a.serial and not a.listen:
        ap.error("укажите --serial (USB-шлюз) и/или --listen (ретранслятор по сети)")
    host, _, port = a.server.rpartition(":")
    recorder = MissionRecorder(a.log_dir, log) if a.log_dir else None
    bridge = Bridge(host or "127.0.0.1", int(port), recorder)
    tasks = []
    if a.listen:
        srv = await asyncio.start_server(bridge.on_client, "0.0.0.0", a.listen)
        log(f"мост: принимаю ретранслятор на TCP {a.listen} -> сервер {bridge.host}:{bridge.port}")
        tasks.append(asyncio.ensure_future(srv.serve_forever()))
    if a.serial:
        from gateway_link import SerialGateway      # pyserial нужен только для USB-шлюза
        log(f"мост: шлюз {a.serial} -> сервер {bridge.host}:{bridge.port}")
        tasks.append(asyncio.ensure_future(SerialGateway(a.serial, a.baud, bridge.on_client, log).run()))
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
