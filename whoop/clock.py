"""Часы программы: настоящие на борту, ускоренные и виртуальные — в проверках на ПК."""
import time


class Clock(object):
    """Настоящее время; speed > 1 ускоряет его (проверка с имитатором и настоящей сетью)."""

    def __init__(self, speed=1.0):
        self.speed = float(speed)
        self._m0 = time.monotonic()

    def time(self):
        return (time.monotonic() - self._m0) * self.speed

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt / self.speed)


class SimClock(object):
    """Виртуальное время: sleep ничего не ждёт, только двигает часы (тесты)."""

    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t

    def sleep(self, dt):
        if dt > 0:
            self.t += dt
