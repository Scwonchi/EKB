"""ROS 2 нода: ровер находит красное окно камерой и подъезжает к нему по оси.

Сервисы (std_srvs/Trigger):
  ~/start      — начать (поиск, подъезд, остановка перед окном)
  ~/stop       — остановить
  ~/calibrate  — подсказать настройки камеры по кадру: окно прямо перед камерой на расстоянии
                 calib_distance_m (от объектива до плоскости окна). Отвечает углом обзора и, если
                 задана высота окна window_bottom_m, поправкой наклона камеры.
Топики:
  ~/status (std_msgs/String, JSON) — состояние, расстояние до окна, смещение оси, угол
  ~/debug/compressed (sensor_msgs/CompressedImage) — выпрямленный кадр с найденным окном
  cmd_vel_topic (geometry_msgs/Twist) — команды движения, по умолчанию /cmd_vel_nav (twist_mux:
                 ручное управление перебивает)
Кадры (frame_source):
  http     — веб-камера на второй Raspberry Pi (webcam_server.py), http_url
  service  — штатная камера ровера, сервис rover_interfaces/GetFrame
  topic    — CompressedImage (у камеры ровера publish_compressed: true)
Одометрия: nav_msgs/Odometry (odom_topic) — сдвиг ровера между кадрами и доводка вблизи окна.
"""
from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .lens import LensParams, Rectifier
from .window_core import (APPROACH, DONE, FAILED, IDLE, ApproachParams, CameraModel, DetectorParams, WindowApproach,
                          WindowGeometry, calibrate_hfov, calibrate_lens_fov, calibrate_pitch, draw_debug,
                          find_window)

try:                                                    # сервис камеры ровера
    from rover_interfaces.srv import GetFrame
except ImportError:                                     # pragma: no cover — сборка без rover_interfaces
    GetFrame = None


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


# Параметры ноды → поля LensParams / WindowGeometry
LENS_PARAMS = {
    'lens_model': 'model', 'camera_hfov_deg': 'hfov_deg', 'camera_pitch_deg': 'pitch_deg',
    'lens_fov_deg': 'fov_deg', 'lens_circle_cx': 'circle_cx', 'lens_circle_cy': 'circle_cy',
    'lens_circle_r': 'circle_r', 'lens_calib_file': 'calib_file', 'rect_width': 'out_width',
    'rect_height': 'out_height', 'rect_hfov_deg': 'out_hfov_deg', 'rect_cy_frac': 'out_cy_frac',
}
GEOM_PARAMS = {
    'window_width_m': 'width_m', 'window_height_m': 'height_m', 'window_frame_m': 'frame_m',
    'window_bottom_m': 'bottom_m', 'window_bottom_min_m': 'bottom_min_m', 'window_bottom_tol_m': 'bottom_tol_m',
    'window_width_tol': 'width_tol',
}


class WindowApproachNode(Node):
    def __init__(self):
        super().__init__('window_approach_node')
        P = self.declare_parameter
        P('frame_source', 'http')                       # http | service | topic
        P('http_url', 'http://rover-cam.local:8080/frame.jpg')
        P('http_timeout_s', 0.5)
        P('max_frame_age_s', 0.5)                       # кадр старше — не для управления
        P('get_frame_service', '/get_frame')
        P('image_topic', '/image_raw/compressed')
        P('odom_topic', '/odom')
        P('cmd_vel_topic', '/cmd_vel_nav')
        P('rate_hz', 10.0)
        P('camera_x_m', 0.05)                           # объектив относительно центра ровера, м
        P('camera_y_m', 0.0)
        P('camera_z_m', 0.45)                           # высота объектива над полом, м
        P('calib_distance_m', 1.5)
        P('publish_debug', True)
        P('debug_rate_hz', 3.0)
        lp = LensParams()
        for name, field in LENS_PARAMS.items():
            P(name, getattr(lp, field))
        g = WindowGeometry()
        for name, field in GEOM_PARAMS.items():
            P(name, getattr(g, field))
        for k, v in vars(ApproachParams()).items():
            P(k, v)
        for k, v in vars(DetectorParams()).items():
            P('red_' + k, v)

        self.approach = WindowApproach(self._approach_params())
        self.rect = Rectifier(self._lens_params())
        self.lock = threading.Lock()
        self.latest_pose = None                         # (WindowPose|None, stamp)
        self.latest_det = None
        self.latest_img = None                          # выпрямленный кадр
        self.cam: Optional[CameraModel] = None
        self.frames = 0
        self.last_frame_t = 0.0
        self.odom_pose: Optional[Tuple[float, float, float]] = None
        self.odom_prev: Optional[Tuple[float, float, float]] = None
        self.request_pending = False
        self.http_error = ''
        self._http_stop = False

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 2)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 20)
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop', self._srv_stop)
        self.create_service(Trigger, '~/calibrate', self._srv_calibrate)

        src = self.get_parameter('frame_source').value
        self.frame_client = None
        rate = float(self.get_parameter('rate_hz').value)
        if src == 'topic':
            self.create_subscription(CompressedImage, self.get_parameter('image_topic').value, self._on_image, 2)
        elif src == 'service':
            if GetFrame is None:
                raise RuntimeError('rover_interfaces не найден: frame_source=service недоступен')
            self.frame_client = self.create_client(GetFrame, self.get_parameter('get_frame_service').value)
        elif src == 'http':
            threading.Thread(target=self._http_loop, args=(rate,), daemon=True).start()
        else:
            raise RuntimeError(f'frame_source: {src}? Нужно http, service или topic')
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.5, self._publish_status)
        self._last_debug = 0.0
        self._last_state = IDLE
        where = self.get_parameter('http_url').value if src == 'http' else src
        self.get_logger().info(f'Подъезд к окну готов: кадры — {where}, камера — {self.rect.p.model}, команды — '
                               f'{self.get_parameter("cmd_vel_topic").value}. Старт: ~/start')

    # ---------- параметры ----------
    def _fill(self, obj, mapping):
        for name, field in mapping.items():
            setattr(obj, field, type(getattr(obj, field))(self.get_parameter(name).value))
        return obj

    def _lens_params(self) -> LensParams:
        return self._fill(LensParams(), LENS_PARAMS)

    def _geometry(self) -> WindowGeometry:
        return self._fill(WindowGeometry(), GEOM_PARAMS)

    def _approach_params(self) -> ApproachParams:
        return self._fill(ApproachParams(), {k: k for k in vars(ApproachParams())})

    def _detector_params(self) -> DetectorParams:
        return self._fill(DetectorParams(), {'red_' + k: k for k in vars(DetectorParams())})

    # ---------- кадры ----------
    def _http_loop(self, rate: float):
        url = self.get_parameter('http_url').value
        timeout = float(self.get_parameter('http_timeout_s').value)
        max_age = float(self.get_parameter('max_frame_age_s').value)
        period = 1.0 / rate
        while not self._http_stop and rclpy.ok():
            t0 = time.monotonic()
            try:
                with urllib.request.urlopen(url, timeout=timeout) as r:
                    data = r.read()
                    age = float(r.headers.get('X-Frame-Age-Ms', '0')) / 1000.0
                age += (time.monotonic() - t0) / 2              # плюс половина времени запроса
                if age <= max_age:
                    self._process_jpeg(data)
                if self.http_error:
                    self.get_logger().info('веб-камера снова отвечает')
                self.http_error = ''
            except (urllib.error.URLError, OSError, ValueError) as e:
                if not self.http_error:
                    self.get_logger().warn(f'веб-камера {url}: {e}')
                self.http_error = str(e)
                time.sleep(0.5)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def _on_image(self, msg: CompressedImage):
        self._process_jpeg(bytes(msg.data))

    def _request_frame(self):
        if self.frame_client is None or self.request_pending:
            return
        if not self.frame_client.service_is_ready():
            return
        self.request_pending = True
        fut = self.frame_client.call_async(GetFrame.Request())
        fut.add_done_callback(self._on_frame_response)

    def _on_frame_response(self, fut):
        self.request_pending = False
        try:
            r = fut.result()
        except Exception as e:                          # noqa: BLE001
            self.get_logger().warn(f'get_frame: {e}', throttle_duration_sec=5.0)
            return
        if r is None or not r.success:
            return
        if r.age_sec > float(self.get_parameter('max_frame_age_s').value):
            return                                      # старый кадр — не годится для управления
        self._process_jpeg(bytes(r.frame.data))

    def _process_jpeg(self, data: bytes):
        src = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if src is None:
            return
        img = self.rect.rectify(src)
        size = (src.shape[1], src.shape[0])
        cam = self.cam
        if cam is None or getattr(self, '_cam_src', None) != size:
            cam = CameraModel.rectified(self.rect, size, float(self.get_parameter('camera_x_m').value),
                                        float(self.get_parameter('camera_y_m').value),
                                        float(self.get_parameter('camera_z_m').value))
            self.cam, self._cam_src = cam, size
        a = self.approach
        prior = a.pose if a.state == APPROACH else None
        det, pose = find_window(img, cam, self._geometry(), self._detector_params(), prior)
        with self.lock:
            self.latest_pose = (pose, time.monotonic())
            self.latest_det = det
            self.latest_img = img
            self.frames += 1
            self.last_frame_t = time.monotonic()

    # ---------- одометрия ----------
    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.odom_pose = (p.position.x, p.position.y, _yaw(p.orientation))

    def _odom_delta(self):
        """Сдвиг ровера с прошлого шага в осях ровера на прошлом шаге."""
        cur = self.odom_pose
        prev, self.odom_prev = self.odom_prev, cur
        if cur is None or prev is None:
            return None
        dxw, dyw = cur[0] - prev[0], cur[1] - prev[1]
        c, s = math.cos(-prev[2]), math.sin(-prev[2])
        dyaw = (cur[2] - prev[2] + math.pi) % (2 * math.pi) - math.pi
        return (c * dxw - s * dyw, s * dxw + c * dyw, dyaw)

    # ---------- цикл ----------
    def _tick(self):
        self._request_frame()
        a = self.approach
        delta = self._odom_delta()
        if a.state in (IDLE, DONE, FAILED):
            return
        meas = None
        with self.lock:
            if self.latest_pose is not None:
                meas, _ = self.latest_pose
                self.latest_pose = None                  # каждый кадр — одно измерение
        if time.monotonic() - self.last_frame_t > 2.0 and not a._blind:
            self._send(0.0, 0.0, 0.0)
            if time.monotonic() - self.last_frame_t > 5.0:
                a.stop('нет кадров с камеры' + (f': {self.http_error}' if self.http_error else ''))
                a.state = FAILED
            return
        cmd = a.update(meas, self.dt, delta)
        self._send(cmd.vx, cmd.vy, cmd.wz)
        if a.state != self._last_state:
            self.get_logger().info(f'{a.state}: {a.message}')
            self._last_state = a.state
            if a.state in (DONE, FAILED):
                for _ in range(3):
                    self._send(0.0, 0.0, 0.0)
        self._publish_debug()

    def _send(self, vx, vy, wz):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(t)

    def _publish_status(self):
        a = self.approach
        st = {'state': a.state, 'message': a.message, 'source': a.source, 'frames': self.frames,
              'frame_age_s': round(time.monotonic() - self.last_frame_t, 2) if self.last_frame_t else None}
        if self.http_error:
            st['camera_error'] = self.http_error
        if a.pose is not None:
            st.update(distance_m=round(a.pose.distance, 3), axis_offset_m=round(a.pose.axis_offset(), 3),
                      yaw_to_face_deg=round(math.degrees(a.pose.yaw_to_face), 1))
        self.status_pub.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if a.state in (IDLE, DONE, FAILED):
            self._publish_debug()                       # отладочный кадр и до старта — навести камеру

    def _publish_debug(self):
        if not self.get_parameter('publish_debug').value:
            return
        now = time.monotonic()
        if now - self._last_debug < 1.0 / float(self.get_parameter('debug_rate_hz').value):
            return
        with self.lock:
            img, det = self.latest_img, self.latest_det
        if img is None:
            return
        self._last_debug = now
        ok, jpg = cv2.imencode('.jpg', draw_debug(img, det, self.approach), [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            m = CompressedImage()
            m.header.stamp = self.get_clock().now().to_msg()
            m.format = 'jpeg'
            m.data = jpg.tobytes()
            self.debug_pub.publish(m)

    # ---------- сервисы ----------
    def _srv_start(self, _req, res):
        self.approach.p = self._approach_params()
        self.approach.start()
        self.odom_prev = None
        self.get_logger().info('Старт: ищу красное окно')
        res.success, res.message = True, 'поиск окна'
        return res

    def _srv_stop(self, _req, res):
        self.approach.stop()
        for _ in range(3):
            self._send(0.0, 0.0, 0.0)
        res.success, res.message = True, 'остановлено'
        return res

    def _srv_calibrate(self, _req, res):
        with self.lock:
            det, cam = self.latest_det, self.cam
        if det is None or cam is None or det.touches_border:
            res.success, res.message = False, 'окно не видно в кадре целиком'
            return res
        g = self._geometry()
        dist = float(self.get_parameter('calib_distance_m').value)
        lp = self.rect.p
        lines = []
        if lp.is_pinhole:
            hfov = calibrate_hfov(det, cam.width, dist, g.height_m)
            lines.append(f'camera_hfov_deg: {hfov:.1f}   (сейчас {lp.hfov_deg:.1f})')
        else:
            fov = calibrate_lens_fov(det, cam, dist, g.height_m, lp.fov_deg)
            lines.append(f'lens_fov_deg: {fov:.1f}   (сейчас {lp.fov_deg:.1f}; повторите 2–3 раза)')
        if g.bottom_m > 0:
            d = calibrate_pitch(det, cam, g)
            lines.append(f'camera_pitch_deg: {lp.pitch_deg + d:.1f}   (сейчас {lp.pitch_deg:.1f})')
        else:
            lines.append('наклон не проверен: задайте window_bottom_m (высоту низа рамки над полом)')
        res.success = True
        res.message = 'Запишите в config/window_approach.yaml и перезапустите ноду: ' + '; '.join(lines)
        self.get_logger().info(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = WindowApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._http_stop = True
        try:
            node._send(0.0, 0.0, 0.0)
        except Exception:                               # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
