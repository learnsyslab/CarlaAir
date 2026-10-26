"""Clock (CARLA game time -> AirSim time) on a synthetic flight; needs numpy and ROS for the import."""
import logging
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carla_sensor_bridge import Clock, Trajectory  # noqa: E402

DELTA = 1_700_000_000_000_000_000  # AirSim ns - game ns


def pose(t: float) -> np.ndarray:
    return np.array([3.0 * math.sin(0.7 * t), 2.0 * math.cos(0.5 * t), 0.3 * t])


def make(seconds: float = 6.0):
    trajectory = Trajectory("unused", 0, "unused")
    for k in range(int(seconds * 250)):
        t = k / 250.0
        trajectory.samples.append((int(t * 1e9) + DELTA, pose(t) + [5.0, 0.0, 0.0], np.eye(3)))
    clock = Clock(trajectory, logging.getLogger("test"))
    clock.next_fit = math.inf  # fits are triggered by hand
    return clock


def feed(clock: Clock, t0: float, t1: float) -> None:
    for t in np.arange(t0, t1, 0.1):
        clock.history.append((float(t), pose(t), np.eye(3)))


def test_fit_locks_on():
    clock = make()
    feed(clock, 0.0, 3.0)
    clock.delta = DELTA + 400_000_000  # first frame: 0.4 s off
    clock._fit()
    assert clock.fitted and abs(clock.delta - DELTA) < 2_000_000


def test_locked_fit_moves_at_most_max_step():
    clock = make()
    feed(clock, 0.0, 3.0)
    clock.delta, clock.fitted = DELTA + 200_000_000, True  # locked, but 0.2 s off
    clock._fit()
    assert clock.stats["clamped"] == 1
    assert abs(clock.delta - (DELTA + 200_000_000 - Clock.MAX_STEP_NS)) < 1_000


def test_reseed_when_no_ground_truth():
    clock = make()
    feed(clock, 0.0, 5.95)  # live: the newest frame is about as old as the newest ground truth
    clock.delta, clock.fitted = DELTA + 30_000_000_000, True  # 30 s off: nothing in the window
    clock.last_fit_ok = time.monotonic() - 5.0
    clock._fit()
    assert clock.stats["reseeded"] == 1 and not clock.fitted
    clock._fit()  # +-1 s search from the re-seeded delta
    assert clock.fitted and abs(clock.delta - DELTA) < 2_000_000


def test_hover_follows_the_clock_drift():
    clock = make()
    rate = 1.4e-3  # game time 0.14 % fast
    for k in range(121):  # good fits over 2 min while flying
        clock.fits.append((float(k), DELTA + int(rate * k * 1e9)))
    clock._fit_drift()
    assert abs(clock.drift - rate) < 1e-6
    # Hovering 3 min later: no fit, the offset keeps following the drift.
    t = 120.0 + 180.0
    stamp = clock.airsim_time(t, pose(0.0), np.eye(3))
    assert abs(stamp - (int(t * 1e9) + DELTA + int(rate * t * 1e9))) < 1_000_000


def test_hover_follows_the_wall_clock():
    """Hovering: delta follows how both clocks move against the wall clock."""
    clock = make()
    airsim_rate, game_rate = 1.0, 1.0 - 1.4e-3  # per wall second; game time 0.14 % slow vs AirSim
    lat_airsim, lat_game = 3_000_000, 25_000_000  # constant poll / delivery latencies cancel
    clock.trajectory.samples.clear()

    def at(wall: float):
        """Ground truth sample and frame stamps at wall time [s]."""
        clock.trajectory.samples.append((DELTA + int(airsim_rate * wall * 1e9), np.zeros(3), np.eye(3),
                                         int(wall * 1e9) + lat_airsim))
        return game_rate * wall

    for k in range(100):  # 2 s before the last good fit
        wall = 10.0 + k * 0.02
        clock.walls.append((at(wall), wall * 1e9 + lat_game - at(wall) * 1e9))
    game = at(12.0)
    clock.delta = DELTA + int((airsim_rate - game_rate) * 12.0 * 1e9)  # the true offset now
    clock.fits.append((game, clock.delta))
    clock.anchor = (clock.delta, *clock._wall_offsets())
    for k in range(1, 9001):  # hover 3 min, frames at 50 Hz, no fit
        wall = 12.0 + k * 0.02
        game = at(wall)
        stamp = clock.airsim_time(game, pose(0.0), np.eye(3), wall + lat_game / 1e9)
    truth = DELTA + int(airsim_rate * wall * 1e9)
    assert abs(stamp - truth) < 1_000_000, (stamp - truth) / 1e6
    assert clock.stats["wall_tracked"] > 0
