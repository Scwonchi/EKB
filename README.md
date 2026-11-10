# EKB — кейс 3 «Операция – спасение»

Хакатон «Технологии ИИ в комплексных беспилотных системах» (Hi-Tech 2026), кейс 3, партнёр —
Роскосмос. Ровер «Сверх» находит камерой красное окно на полигоне и встаёт так, чтобы его ось
проходила через окно. С площадки на ровере взлетает микродрон, пролетает окно, находит в зоне
поиска пострадавших (1–3: манекены, фигуры, плакаты) и шлёт тревогу с координатами в системе
полигона на станцию оператора.

**Состояние:** подъезд ровера к окну проверен в имитации и тестами без ROS; программы
микродрона и автомат миссии — в проекте (см. «Документы»); на железе пока ничего не испытано.

## Состав

| Папка | Что | Где работает |
|---|---|---|
| [`rover/rover_window_approach/`](rover/rover_window_approach/README.md) | подъезд ровера к окну по оси: распознавание красной рамки, выпрямление кадра, регулятор для меканума, веб-камера на второй Raspberry Pi | ровер «Сверх» (ROS 2 Jazzy), вторая Raspberry Pi |
| [`radar_nn/`](radar_nn/README.md) | разведчик и капсула с радарами LD2450 / LD2410C: сканирование, нейросеть на борту против ложных целей, карта обнаружения | ESP32-S3, XIAO ESP32S3 Sense |
| [`telemetry/`](telemetry/README.md) | протокол «устройство ↔ сервер» на protobuf (Syn/Ack + снимки), ретранслятор, учебный сервер, клиент для iot-go | ESP32, ПК |

Как микродрон взаимодействует с ровером (взлёт с площадки, пролёт окна, пересчёт координат,
протокол связи) — в документе «Кейс 3: микродрон взлетает с ровера» (ссылка ниже).

## Быстрый старт

Подъезд ровера к окну — пакет ROS 2, кладётся в рабочее пространство ровера:

```bash
cp -r rover/rover_window_approach ~/sverk_rover/src/motion/
cd ~/sverk_rover && colcon build --packages-select rover_window_approach
source install/setup.bash
ros2 launch rover_window_approach window_approach.launch.py
ros2 service call /window_approach_node/start std_srvs/srv/Trigger
```

Проверка без ровера и без ROS (нужны Python 3, OpenCV, NumPy):

```bash
cd rover/rover_window_approach
python3 -m rover_window_approach.sim     # 27 попыток подъезда в имитации
python3 -m pytest test
```

Прошивки ESP32 собираются в Arduino IDE или arduino-cli (ядро ESP32 3.x), библиотека
`telemetry/esp/libraries/TelemetryProto` копируется в папку библиотек Arduino. Подробности и
схемы подключения — в README каждой папки.

## Что сделано командой, а что взято готовым

| Готовое (из поставки и открытые библиотеки) | Сделано командой |
|---|---|
| ровер «Сверх»: ПО ровера (ROS 2, Nav2, SLAM, драйверы, камера) — репозиторий организаторов | пакет `rover_window_approach` |
| микродрон «Сверх»: PX4, `simple_offboard_py`, детектор YOLO на борту | программы микродрона и автомат миссии (в работе) |
| сервер iot-go | клиент iot-go, протокол Syn/Ack на стороне устройств |
| nanopb 0.4.9.2 (среда выполнения protobuf для ESP32, лицензия zlib) | схемы `.proto`, кодек, `TelemetryLink`, ретранслятор, шлюз |
| OpenCV, NumPy, Arduino-ESP32 | прошивки разведчика и капсулы, нейросети радаров и их обучение (`train.py`, `tools/`) |

## Библиотеки и инструменты

- Ровер: Python 3, ROS 2 Jazzy (rclpy, std_srvs, sensor_msgs, nav_msgs, geometry_msgs),
  OpenCV (в том числе `cv2.fisheye`), NumPy.
- Устройства: Arduino-ESP32 3.x, nanopb 0.4.9.2, easyp (генерация кода из `.proto`).
- Сервер и инструменты: Python 3, protobuf, pyserial; C++17 для имитаторов на ПК.

## Документы

- Кейс 3: микродрон взлетает с ровера — схема взаимодействия:
  https://claude.ai/artifact/V21DfhBy5FjxpzB64EBoD6
- Кейс 3: сверка решения с заданием и критериями: https://claude.ai/artifact/KiJqtz4m18CKQRPRD2RwGH
- Документация микродрона и ровера «Сверх»: https://edu.sverk.tech/learn/whoop/introduction,
  https://edu.sverk.tech/learn/rover/readme

## Лицензия

MIT — см. [LICENSE](LICENSE). nanopb в `telemetry/esp/libraries/TelemetryProto/src/pb*` —
под своей лицензией (zlib).

Пароли Wi-Fi в скетчах — заглушки (`12345678`, `router-password`): настоящие в репозиторий
не коммитить.
