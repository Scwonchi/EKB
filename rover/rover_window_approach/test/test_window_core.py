"""Проверки без ROS: веб-камера и «рыбий глаз», распознавание, отсев помех, калибровка, подъезд."""
import math
import threading
from dataclasses import replace
import time
import urllib.request

import cv2
import numpy as np
import pytest

from rover_window_approach import sim
from rover_window_approach.lens import LensParams, Rectifier, find_image_circle, save_calibration
from rover_window_approach.lens_calib import calibrate
from rover_window_approach.window_core import (ApproachParams, CameraModel, Command, FAILED, WindowApproach,
                                               calibrate_hfov, calibrate_lens_fov, calibrate_pitch, detect_window,
                                               find_window)

CAMERAS = {'webcam': sim.Camera(sim.WEBCAM, 1), 'fisheye': sim.Camera(sim.FISHEYE, 1)}


def _see(robot, scene=None, setup=None, kind='webcam'):
    camera = CAMERAS[kind]
    lens = camera.lens
    scene = scene or sim.Scene()
    setup = setup or sim.default_setup(scene, lens)
    rect = Rectifier(setup.lens)
    img = rect.rectify(camera.render(scene, robot))
    cam = CameraModel.rectified(rect, (lens.width, lens.height), setup.cam_x, 0.0, setup.cam_h)
    return img, cam, find_window(img, cam, setup.geom)


def _truth(robot, scene):
    """Расстояние до плоскости окна и поворот к нему (рад) — по-настоящему."""
    nx, ny = math.cos(scene.window_yaw), math.sin(scene.window_yaw)
    dist = (robot[0] - scene.window_x) * nx + (robot[1] - scene.window_y) * ny
    turn = (math.atan2(-ny, -nx) - robot[2] + math.pi) % (2 * math.pi) - math.pi
    return dist, turn


# ---------------------------------------------------------------- выпрямление


@pytest.mark.parametrize('kind', ['webcam', 'fisheye'])
def test_rectified_horizon_is_level(kind):
    """Камера наклонена (веб-камера на 15°, «рыбий глаз» на 4°); в выпрямленном кадре горизонт —
    горизонтальная линия на высоте главной точки."""
    camera = CAMERAS[kind] if kind == 'fisheye' else sim.Camera(replace(sim.WEBCAM, pitch_deg=15.0), 1)
    rect = Rectifier(sim.default_setup(sim.Scene(), camera.lens).lens)
    img = rect.rectify(camera.bg)
    _, h, _, _, cy = rect.out_camera((camera.lens.width, camera.lens.height))
    rows = []
    for x in (150, 320, 490):
        col = img[:, x, 0].astype(int)
        inside = (col[:-1] > 80) & (col[1:] > 80)                      # чёрное — за краем исходного кадра
        rows.append(int(np.argmax((np.abs(np.diff(col)) > 20) & inside)))
    assert max(rows) - min(rows) <= 2
    assert abs(np.mean(rows) - cy) <= 3


def test_image_circle():
    c = find_image_circle(CAMERAS['fisheye'].bg)
    assert c is not None
    assert c[0] == pytest.approx(639.5, abs=1.5) and c[2] == pytest.approx(sim.FISHEYE.circle_r, abs=3)


# ---------------------------------------------------------------- положение окна


@pytest.mark.parametrize('kind,robot,window_yaw', [
    ('webcam', (2.0, 0.0, math.pi), 0.0), ('webcam', (2.5, 0.6, math.pi), 0.3),
    ('webcam', (1.8, -0.5, math.pi - 0.25), -0.3), ('webcam', (0.8, 0.05, math.pi), 0.0),
    ('fisheye', (2.0, 0.0, math.pi), 0.0), ('fisheye', (2.5, 0.6, math.pi), 0.3),
    ('fisheye', (1.4, 0.7, math.pi - 0.15), 0.0)])                     # окно у края кадра
def test_pose_estimate(kind, robot, window_yaw):
    scene = sim.Scene(window_yaw=window_yaw)
    _, _, (det, p) = _see(robot, scene, kind=kind)
    assert p is not None
    dist, turn = _truth(robot, scene)
    assert p.distance == pytest.approx(dist, rel=0.06)
    assert math.degrees(p.yaw_to_face) == pytest.approx(math.degrees(turn), abs=5)


def test_floor_frame_of_same_size_rejected():
    """Ближе окна стоит такая же красная рамка, но на полу: самая крупная рамка в кадре — она,
    а выбрано окно (по высоте над полом)."""
    scene = sim.Scene()
    robot = (1.2, -1.3, math.pi - 0.25)
    img, cam, (det, p) = _see(robot, scene, kind='fisheye')
    biggest = detect_window(img)
    assert p is not None
    assert np.abs(biggest.corners - det.corners).max() > 20          # крупнейшая рамка — не окно
    dist, _ = _truth(robot, scene)
    assert p.distance == pytest.approx(dist, rel=0.08)


def test_wrong_size_rejected():
    """Окно задано вдвое шире настоящего — найденная рамка не подходит."""
    scene = sim.Scene(distractors=False)
    setup = sim.default_setup(scene, sim.WEBCAM)
    setup.geom.width_m = 2.0
    _, _, (det, p) = _see((2.0, 0.0, math.pi), scene, setup)
    assert p is None


def test_window_too_close_no_pose():
    """Вплотную окно уходит за верх кадра — позы нет (иначе неверное расстояние)."""
    _, _, (det, p) = _see((0.5, 0.0, math.pi))
    assert p is None


def test_no_window_blank():
    assert detect_window(np.full((480, 640, 3), 120, np.uint8)) is None


# ---------------------------------------------------------------- калибровка


def test_calibrate_webcam_hfov_and_pitch():
    """Угол обзора задан 78° вместо 70°, наклон 22° вместо 30°: по кадру с окном на 1,5 м
    калибровка возвращает почти верные значения (за 3 прохода)."""
    scene = sim.Scene(distractors=False)
    setup = sim.default_setup(scene, sim.WEBCAM)
    setup.lens.hfov_deg, setup.lens.pitch_deg = 78.0, 22.0
    dist = 1.5
    robot = (dist + sim.WEBCAM.cam_x, 0.0, math.pi)
    for _ in range(3):
        _, cam, (det, p) = _see(robot, scene, setup)
        assert det is not None
        setup.lens.hfov_deg = calibrate_hfov(det, cam.width, dist, scene.height)
        _, cam, (det, p) = _see(robot, scene, setup)
        setup.lens.pitch_deg += calibrate_pitch(det, cam, setup.geom)
    assert setup.lens.hfov_deg == pytest.approx(70.0, abs=1.5)
    assert setup.lens.pitch_deg == pytest.approx(30.0, abs=1.0)


def test_chessboard_calibration(tmp_path):
    frames = sim.chessboard_frames(sim.FISHEYE, n=14, seed=2)
    rms, K, D, size, used = calibrate(frames, 9, 6, 0.04)
    assert rms < 1.0 and len(used) >= 10
    assert K[0, 0] == pytest.approx(sim.FISHEYE.K[0, 0], rel=0.01)
    path = str(tmp_path / 'fe.yaml')
    save_calibration(path, K, D, size, rms)
    scene = sim.Scene(window_yaw=0.3)
    setup = sim.default_setup(scene, sim.FISHEYE)
    setup.lens = LensParams(model='calibrated', calib_file=path, pitch_deg=sim.FISHEYE.pitch_deg)
    robot = (2.0, 0.5, math.pi - 0.2)
    _, _, (det, p) = _see(robot, scene, setup, kind='fisheye')
    dist, _ = _truth(robot, scene)
    assert p is not None and p.distance == pytest.approx(dist, rel=0.05)


def test_calibrate_lens_fov_moves_toward_truth():
    """Угол «рыбьего глаза» задан 170° вместо 190°: калибровка по окну на 2,5 м подсказывает ближе к 190°."""
    scene = sim.Scene(distractors=False)
    setup = sim.default_setup(scene, sim.FISHEYE)
    setup.lens.fov_deg = 170.0
    robot = (2.5 + sim.FISHEYE.cam_x, 0.0, math.pi)
    _, cam, (det, p) = _see(robot, scene, setup, kind='fisheye')
    fov = calibrate_lens_fov(det, cam, 2.5, scene.height, 170.0)
    assert abs(fov - sim.FISHEYE.fov_deg) < abs(170.0 - sim.FISHEYE.fov_deg) / 2


# ---------------------------------------------------------------- веб-камера по HTTP


def test_webcam_server_serves_fresh_frames(tmp_path):
    """webcam_server читает «камеру» (здесь — видеофайл) и отдаёт кадры с возрастом кадра."""
    from rover_window_approach import webcam_server as ws
    path = str(tmp_path / 'cam.avi')
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
    if not vw.isOpened():
        pytest.skip('нет кодека MJPG')
    for i in range(30):
        vw.write(np.full((480, 640, 3), (i * 8) % 255, np.uint8))
    vw.release()
    g = ws.Grabber(path, 640, 480, 30, '', 0, 80)
    from http.server import ThreadingHTTPServer
    srv = ThreadingHTTPServer(('127.0.0.1', 0), ws.make_handler(g))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        url = f'http://127.0.0.1:{srv.server_address[1]}'
        for _ in range(50):
            if g.jpeg() is not None:
                break
            time.sleep(0.05)
        with urllib.request.urlopen(url + '/frame.jpg', timeout=2) as r:
            data = r.read()
            age = float(r.headers['X-Frame-Age-Ms'])
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert img.shape == (480, 640, 3)
        assert age < 1000
        with urllib.request.urlopen(url + '/health', timeout=2) as r:
            assert b'"size": [640, 480]' in r.read()
    finally:
        g.running = False
        srv.shutdown()


# ---------------------------------------------------------------- подъезд


@pytest.mark.parametrize('start', [(2.5, 1.0, math.pi), (2.0, -0.8, math.pi - 0.6), (2.5, 0.0, 0.0)])
def test_closed_loop_webcam(start):
    scene = sim.Scene(window_yaw=0.2)
    r = sim.run(start, scene, sim.WEBCAM, seed=3, camera=CAMERAS['webcam'])
    assert r.passes, r.message
    assert abs(r.axis_offset_m) < 0.05
    assert abs(r.yaw_err_deg) < 3


def test_closed_loop_fisheye():
    scene = sim.Scene(window_yaw=-0.2)
    r = sim.run((2.5, -0.5, math.pi), scene, sim.FISHEYE, seed=4, camera=CAMERAS['fisheye'])
    assert r.passes, r.message


def test_search_fails_without_window():
    ap = WindowApproach(ApproachParams(search_timeout_s=2.0))
    ap.start()
    for _ in range(40):
        ap.update(None, 0.1)
    assert ap.state == FAILED


def test_stop_command_after_done():
    ap = WindowApproach()
    assert ap.update(None, 0.1) == Command()      # не запущен — стоим
