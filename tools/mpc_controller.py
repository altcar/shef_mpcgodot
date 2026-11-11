#!/usr/bin/env python3
"""TCP MPC controller for the Godot lane-following simulation.

This is a direct port of `untitled.m` / `Copy_of_MPCdemo_utf8.m`, with the
transport changed from UDP to TCP. The packet layout is *exactly* the same:

    sensor frame    5584 bytes = 698 little-endian doubles
    control frame     16 bytes =   2 little-endian doubles (steer, speed)

TCP is a stream, so frames are delimited by their fixed size: the controller
reads exactly 5584 bytes and writes exactly 16 bytes. No header is added.

The Godot Web build cannot open TCP sockets, so connect this controller to
`tools/bridge.py` (default 127.0.0.1:4248), not directly to the browser.

Example:
    python tools/bridge.py --web-root build/web
    python tools/mpc_controller.py --host 127.0.0.1 --port 4248 --speed 15

The NMPC objective and weights match the MATLAB implementation. If SciPy is
installed, SLSQP is used (the closest equivalent to MATLAB's `fmincon`);
otherwise a small projected-gradient solver is used, so NumPy/SciPy are not
required.
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Sequence

SENSOR_DOUBLES = 698
SENSOR_BYTES = SENSOR_DOUBLES * 8
CONTROL_BYTES = 16

STATE_COUNT = 13
LIDAR_COUNT = 360
LANE_HORIZONS = 5
LANE_BINS = 64
CTE_COUNT = 5

assert STATE_COUNT + LIDAR_COUNT + LANE_HORIZONS * LANE_BINS + CTE_COUNT == SENSOR_DOUBLES

# MATLAB controller constants.
LANE_WIDTH = 3.5
VEHICLE_WIDTH = 2.0
WHEEL_MARGIN = 0.10
MAX_CENTER_ERROR = (LANE_WIDTH - VEHICLE_WIDTH) / 2.0 - WHEEL_MARGIN

HORIZON_STEP_M = 2.5
NMPC_STEPS = 5
MAX_YAW_RATE = 1.5

W_CTE = 100.0
W_U = 5.0
W_DU = 50.0
W_EDGE = 1200.0


@dataclass(frozen=True)
class SensorFrame:
    state: tuple[float, ...]
    lidar: tuple[float, ...]
    lane: tuple[tuple[float, ...], ...]
    cte: tuple[float, ...]


def parse_sensor(packet: bytes) -> SensorFrame:
    """Decode one 5584-byte sensor packet."""
    if len(packet) != SENSOR_BYTES:
        raise ValueError(f"sensor packet must be {SENSOR_BYTES} bytes, got {len(packet)}")
    values = struct.unpack("<698d", packet)

    state = values[:STATE_COUNT]
    lidar = values[STATE_COUNT : STATE_COUNT + LIDAR_COUNT]
    lane_start = STATE_COUNT + LIDAR_COUNT
    lane = tuple(
        values[lane_start + horizon * LANE_BINS : lane_start + (horizon + 1) * LANE_BINS]
        for horizon in range(LANE_HORIZONS)
    )
    cte = values[SENSOR_DOUBLES - CTE_COUNT :]
    return SensorFrame(state=state, lidar=lidar, lane=lane, cte=cte)


def matlab_round(value: float) -> float:
    """MATLAB rounds halves away from zero; Python rounds halves to even."""
    if value >= 0.0:
        return float(math.floor(value + 0.5))
    return float(math.ceil(value - 0.5))


def compute_nmpc_cost(
    yaw_rates: Sequence[float],
    cte_reference: Sequence[float],
    dt: float,
    speed: float,
    last_steer: float,
    max_center_error: float = MAX_CENTER_ERROR,
) -> float:
    """Unicycle prediction + lane-keeping cost, matching the MATLAB version."""
    y = 0.0
    theta = 0.0
    cost = 0.0
    previous = last_steer

    for index, yaw_rate in enumerate(yaw_rates):
        theta += yaw_rate * dt
        y += speed * math.sin(theta) * dt

        reference = cte_reference[index]
        error = y - reference
        edge_excess = max(0.0, abs(error) - max_center_error)

        cost += W_CTE * error * error
        cost += W_U * yaw_rate * yaw_rate
        cost += W_DU * (yaw_rate - previous) ** 2
        cost += W_EDGE * edge_excess * edge_excess
        previous = yaw_rate

    return cost


def _project(values: list[float], lower: float, upper: float) -> list[float]:
    return [min(max(value, lower), upper) for value in values]


def _numeric_gradient(function, values: list[float], epsilon: float = 1e-4) -> list[float]:
    gradient: list[float] = []
    for index in range(len(values)):
        step = epsilon * max(1.0, abs(values[index]))
        plus = list(values)
        minus = list(values)
        plus[index] += step
        minus[index] -= step
        gradient.append((function(plus) - function(minus)) / (2.0 * step))
    return gradient


def _optimize_pgd(function, initial: Sequence[float], lower: float, upper: float, iterations: int = 80):
    """Projected gradient descent with an adaptive step. Dependency-free."""
    values = _project(list(initial), lower, upper)
    best = function(values)
    step = 0.5

    for _ in range(iterations):
        gradient = _numeric_gradient(function, values)
        norm = math.sqrt(sum(component * component for component in gradient))
        if norm < 1e-12:
            break

        improved = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = _project(
                [values[i] - step * scale * gradient[i] / norm for i in range(len(values))],
                lower,
                upper,
            )
            candidate_cost = function(candidate)
            if candidate_cost < best - 1e-12:
                values = candidate
                best = candidate_cost
                step = min(step * 1.5, 2.0)
                improved = True
                break

        if not improved:
            step *= 0.5
            if step < 1e-4:
                break

    return values, best


def _optimize_slsqp(function, initial: Sequence[float], lower: float, upper: float):
    from scipy.optimize import minimize  # imported lazily on purpose

    result = minimize(
        function,
        list(initial),
        method="SLSQP",
        bounds=[(lower, upper)] * len(initial),
        options={"maxiter": 100, "ftol": 1e-6, "disp": False},
    )
    if result.success and all(math.isfinite(value) for value in result.x):
        return list(result.x), float(result.fun)
    return _optimize_pgd(function, initial, lower, upper)


def lane_following_nmpc(
    state: Sequence[float],
    cte_horizon: Sequence[float],
    speed: float,
    last_steer: float,
    optimizer: str = "auto",
) -> float:
    """Return the first yaw-rate control action (rad/s)."""
    if speed <= 0.1:
        return 0.0

    count = min(len(cte_horizon), NMPC_STEPS)
    reference = list(cte_horizon[:count])
    dt = HORIZON_STEP_M / speed

    def objective(yaw_rates):
        return compute_nmpc_cost(yaw_rates, reference, dt, speed, last_steer)

    initial = [last_steer] * count
    selected = optimizer
    if selected == "auto":
        try:
            import scipy  # noqa: F401

            selected = "scipy"
        except ImportError:
            selected = "pgd"

    if selected == "scipy":
        yaw_rates, _cost = _optimize_slsqp(objective, initial, -MAX_YAW_RATE, MAX_YAW_RATE)
    else:
        yaw_rates, _cost = _optimize_pgd(objective, initial, -MAX_YAW_RATE, MAX_YAW_RATE)

    return float(min(max(yaw_rates[0], -MAX_YAW_RATE), MAX_YAW_RATE))


class SensorStream:
    """Read fixed-size sensor frames from a TCP stream, always returning the newest."""

    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buffer = bytearray()

    def next_frame(self, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout

        while len(self.buffer) < SENSOR_BYTES:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError("timed out waiting for a sensor frame")
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                raise TimeoutError("timed out waiting for a sensor frame")
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("controller socket closed")
            self.buffer.extend(chunk)

        # Drain whatever else has already arrived and keep only the newest frame.
        while True:
            readable, _, _ = select.select([self.sock], [], [], 0.0)
            if not readable:
                break
            chunk = self.sock.recv(65536)
            if not chunk:
                break
            self.buffer.extend(chunk)

        complete_frames = len(self.buffer) // SENSOR_BYTES
        if complete_frames > 1:
            del self.buffer[: (complete_frames - 1) * SENSOR_BYTES]

        frame = bytes(self.buffer[:SENSOR_BYTES])
        del self.buffer[:SENSOR_BYTES]
        return frame


class LivePlot:
    """Small optional matplotlib view of the same four panels as the MATLAB monitor."""

    def __init__(self) -> None:
        import matplotlib.pyplot as plt
        import numpy as np

        self.plt = plt
        self.np = np
        self.steer_history = deque(maxlen=400)
        self.last_draw = 0.0

        plt.ion()
        self.fig = plt.figure(figsize=(11, 7))
        self.fig.suptitle("MPC TCP Controller")

        self.lidar_ax = self.fig.add_subplot(2, 2, 1, projection="polar")
        self.lidar_line, = self.lidar_ax.plot([], [], linewidth=1.0, color="tab:blue")
        self.lidar_ax.set_ylim(0, 50)
        self.lidar_ax.set_title("LiDAR 360")

        self.lane_ax = self.fig.add_subplot(2, 2, 2)
        self.lane_lines = [
            self.lane_ax.plot([], [], linewidth=1.2, label=f"H={index * 5}m")[0]
            for index in range(LANE_HORIZONS)
        ]
        self.lane_ax.set_xlim(-4, 4)
        self.lane_ax.set_ylim(-0.1, 1.1)
        self.lane_ax.set_xlabel("Lateral offset (m)")
        self.lane_ax.set_ylabel("Intensity")
        self.lane_ax.set_title("Lane patterns")
        self.lane_ax.legend(loc="best", fontsize=7)
        self.lane_ax.grid(True, alpha=0.3)

        self.cte_ax = self.fig.add_subplot(2, 2, 3)
        self.cte_line, = self.cte_ax.plot(
            [0, 2.5, 5.0, 7.5, 10.0], [0.0] * 5, "r-o", linewidth=1.5
        )
        self.cte_ax.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
        self.cte_ax.set_xlabel("Look-ahead distance (m)")
        self.cte_ax.set_ylabel("CTE (m)")
        self.cte_ax.set_title("CTE horizon")
        self.cte_ax.set_ylim(-3, 3)
        self.cte_ax.grid(True, alpha=0.3)

        self.steer_ax = self.fig.add_subplot(2, 2, 4)
        self.steer_line, = self.steer_ax.plot([], [], color="magenta", linewidth=1.2)
        self.steer_ax.set_xlim(0, 400)
        self.steer_ax.set_ylim(-1.6, 1.6)
        self.steer_ax.set_xlabel("frame")
        self.steer_ax.set_ylabel("yaw rate (rad/s)")
        self.steer_ax.set_title("Steering history")
        self.steer_ax.grid(True, alpha=0.3)

        self.fig.tight_layout()
        self.fig.show()

    def update(self, frame: SensorFrame, steer: float, force: bool = False) -> None:
        now = time.monotonic()
        self.steer_history.append(steer)
        if not force and now - self.last_draw < 0.08:
            return
        self.last_draw = now

        angles = self.np.deg2rad(self.np.arange(LIDAR_COUNT))
        self.lidar_line.set_data(angles, self.np.asarray(frame.lidar))

        lateral = self.np.linspace(-4.0, 4.0, LANE_BINS)
        for line, values in zip(self.lane_lines, frame.lane):
            line.set_data(lateral, self.np.asarray(values))

        self.cte_line.set_ydata(self.np.asarray(frame.cte))
        self.steer_line.set_data(
            self.np.arange(len(self.steer_history)), self.np.asarray(self.steer_history)
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        try:
            self.plt.pause(0.001)
        except Exception:
            pass


def self_check() -> int:
    import random

    values = [random.uniform(-1.0, 1.0) for _ in range(SENSOR_DOUBLES)]
    frame = parse_sensor(struct.pack("<698d", *values))
    assert len(frame.state) == STATE_COUNT
    assert len(frame.lidar) == LIDAR_COUNT
    assert len(frame.lane) == LANE_HORIZONS
    assert all(len(row) == LANE_BINS for row in frame.lane)
    assert len(frame.cte) == CTE_COUNT

    steer = lane_following_nmpc(frame.state, [0.1, 0.2, 0.3, 0.4, 0.5], 15.0, 0.0)
    assert math.isfinite(steer)
    assert abs(steer) <= MAX_YAW_RATE

    cost = compute_nmpc_cost([0.1] * 5, [0.0] * 5, 2.5 / 15.0, 15.0, 0.0)
    assert math.isfinite(cost) and cost >= 0.0

    print("self-check ok")
    return 0


def run(args: argparse.Namespace) -> int:
    try:
        sock = socket.create_connection((args.host, args.port), timeout=args.connect_timeout)
    except OSError as exc:
        print(f"cannot connect to bridge at {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 1

    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(None)
    stream = SensorStream(sock)

    speed_command = float(args.speed)
    last_steer = 0.0
    started = time.monotonic()
    last_status = 0.0
    packet_count = 0

    plot = LivePlot() if args.plot else None

    # Send a first command immediately so the car starts even before the first frame.
    sock.sendall(struct.pack("<dd", last_steer, speed_command))
    print(
        f"connected to {args.host}:{args.port}; speed={speed_command:.2f} m/s; "
        f"optimizer={args.optimizer}; Ctrl+C to stop"
    )

    try:
        while True:
            if args.duration > 0.0 and time.monotonic() - started >= args.duration:
                break

            packet = stream.next_frame(args.timeout)
            frame = parse_sensor(packet)
            packet_count += 1

            # Snap to the current lane, exactly like the MATLAB controller.
            current_lane_offset = matlab_round(frame.cte[0] / LANE_WIDTH) * LANE_WIDTH
            snapped_horizon = [value - current_lane_offset for value in frame.cte]

            last_steer = lane_following_nmpc(
                frame.state,
                snapped_horizon,
                speed_command,
                last_steer,
                optimizer=args.optimizer,
            )
            sock.sendall(struct.pack("<dd", last_steer, speed_command))

            now = time.monotonic()
            if not args.quiet and now - last_status >= args.log_interval:
                last_status = now
                minimum_lidar = min(frame.lidar)
                print(
                    f"t={now - started:7.1f}s packets={packet_count:6d} "
                    f"pos=({frame.state[0]:7.2f},{frame.state[2]:7.2f}) "
                    f"cte={frame.cte[0]:+6.3f}m steer={last_steer:+6.3f}rad/s "
                    f"lidar_min={minimum_lidar:6.2f}m"
                )

            if plot is not None:
                plot.update(frame, last_steer)

            if args.max_packets > 0 and packet_count >= args.max_packets:
                break
    except (TimeoutError, ConnectionError, ValueError, OSError) as exc:
        print(f"\ncontroller stopped: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\ncontroller stopped")
    finally:
        try:
            sock.close()
        except OSError:
            pass

    return 0


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="TCP NMPC lane-following controller for the Godot MPC simulation.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--host", default="127.0.0.1", help="bridge TCP host")
    parser.add_argument("--port", type=int, default=4248, help="bridge TCP controller port")
    parser.add_argument("--speed", type=float, default=15.0, help="constant speed command (m/s)")
    parser.add_argument("--timeout", type=float, default=30.0, help="sensor read timeout (seconds)")
    parser.add_argument("--connect-timeout", type=float, default=5.0, help="TCP connect timeout (seconds)")
    parser.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    parser.add_argument("--max-packets", type=int, default=0, help="stop after N sensor frames (0 = forever)")
    parser.add_argument(
        "--optimizer",
        choices=("auto", "scipy", "pgd"),
        default="auto",
        help="auto uses SciPy when installed, otherwise projected gradient descent",
    )
    parser.add_argument("--plot", action="store_true", help="show live matplotlib panels")
    parser.add_argument("--quiet", action="store_true", help="disable periodic status lines")
    parser.add_argument("--log-interval", type=float, default=0.5, help="seconds between status lines")
    parser.add_argument("--check", action="store_true", help="run an offline packet/optimizer self-check and exit")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    if args.check:
        return self_check()
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
