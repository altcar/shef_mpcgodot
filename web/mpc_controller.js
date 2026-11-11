/*
 * Browser-side MPC controller.
 *
 * This is the JavaScript port of the MATLAB NMPC lane-following controller.
 * It runs inside the same page as the Godot Web build, so GitHub Pages needs
 * no server, no WebSocket, and no TCP bridge.
 *
 * The Godot transport calls:
 *   MPCController.submit(Float64Array[18])   // 13 state + 5 CTE values
 *   MPCController.getSteering()              // yaw rate, rad/s
 *   MPCController.getSpeed()                 // constant speed, m/s
 *
 * Optional URL parameters:
 *   ?speed=15     set the constant speed command
 */
(function () {
  "use strict";

  const LANE_WIDTH = 3.5;
  const VEHICLE_WIDTH = 2.0;
  const WHEEL_MARGIN = 0.10;
  const MAX_CENTER_ERROR = (LANE_WIDTH - VEHICLE_WIDTH) / 2.0 - WHEEL_MARGIN;

  const HORIZON_STEP_M = 2.5;
  const MAX_YAW_RATE = 1.5;

  const W_CTE = 100.0;
  const W_U = 5.0;
  const W_DU = 50.0;
  const W_EDGE = 1200.0;

  // MATLAB rounds halves away from zero; JavaScript Math.round does not.
  function matlabRound(value) {
    return value >= 0.0 ? Math.floor(value + 0.5) : Math.ceil(value - 0.5);
  }

  function computeNmpcCost(yawRates, cteReference, dt, speed, lastSteer) {
    let y = 0.0;
    let theta = 0.0;
    let cost = 0.0;
    let previous = lastSteer;

    for (let index = 0; index < yawRates.length; index++) {
      const yawRate = yawRates[index];
      theta += yawRate * dt;
      y += speed * Math.sin(theta) * dt;

      const error = y - cteReference[index];
      const edgeExcess = Math.max(0.0, Math.abs(error) - MAX_CENTER_ERROR);

      cost += W_CTE * error * error;
      cost += W_U * yawRate * yawRate;
      cost += W_DU * (yawRate - previous) * (yawRate - previous);
      cost += W_EDGE * edgeExcess * edgeExcess;
      previous = yawRate;
    }

    return cost;
  }

  function project(values, lower, upper) {
    return values.map((value) => Math.min(Math.max(value, lower), upper));
  }

  function numericGradient(fn, values, epsilon) {
    const gradient = new Array(values.length);
    for (let index = 0; index < values.length; index++) {
      const step = epsilon * Math.max(1.0, Math.abs(values[index]));
      const plus = values.slice();
      const minus = values.slice();
      plus[index] += step;
      minus[index] -= step;
      gradient[index] = (fn(plus) - fn(minus)) / (2.0 * step);
    }
    return gradient;
  }

  // Projected gradient descent with an adaptive step. Dependency-free and
  // small enough to run on the main thread at 20 Hz.
  function optimizeProjectedGradient(fn, initial, lower, upper, iterations) {
    let values = project(initial, lower, upper);
    let best = fn(values);
    let step = 0.5;
    const limit = iterations || 80;

    for (let iteration = 0; iteration < limit; iteration++) {
      const gradient = numericGradient(fn, values, 1e-4);
      const norm = Math.hypot.apply(Math, gradient);
      if (norm < 1e-12) {
        break;
      }

      let improved = false;
      const scales = [1.0, 0.5, 0.25, 0.1];
      for (let scaleIndex = 0; scaleIndex < scales.length; scaleIndex++) {
        const scale = scales[scaleIndex];
        const candidate = project(
          values.map((value, index) => value - (step * scale * gradient[index]) / norm),
          lower,
          upper
        );
        const candidateCost = fn(candidate);
        if (candidateCost < best - 1e-12) {
          values = candidate;
          best = candidateCost;
          step = Math.min(step * 1.5, 2.0);
          improved = true;
          break;
        }
      }

      if (!improved) {
        step *= 0.5;
        if (step < 1e-4) {
          break;
        }
      }
    }

    return values;
  }

  class LaneController {
    constructor(speed) {
      this.speed = speed;
      this.lastSteer = 0.0;
      this.steer = 0.0;
      this.packets = 0;
      this.lastCte = 0.0;
      this.lastSubmit = 0.0;
    }

    submit(values) {
      if (typeof values === "string") {
        try {
          values = JSON.parse(values);
        } catch (error) {
          return;
        }
      }
      if (!values || values.length < 18) {
        return;
      }

      // Layout: 0..12 = state, 13..17 = CTE horizon.
      const cte = Array.from(values.slice(13, 18));
      if (!cte.every(Number.isFinite)) {
        return;
      }

      const speed = Math.max(this.speed, 0.1);
      const laneOffset = matlabRound(cte[0] / LANE_WIDTH) * LANE_WIDTH;
      const reference = cte.map((value) => value - laneOffset);
      const dt = HORIZON_STEP_M / speed;
      const lastSteer = this.lastSteer;

      const objective = (yawRates) =>
        computeNmpcCost(yawRates, reference, dt, speed, lastSteer);
      const initial = new Array(reference.length).fill(lastSteer);
      const optimized = optimizeProjectedGradient(
        objective,
        initial,
        -MAX_YAW_RATE,
        MAX_YAW_RATE,
        80
      );

      this.steer = Math.min(Math.max(optimized[0], -MAX_YAW_RATE), MAX_YAW_RATE);
      this.lastSteer = this.steer;
      this.lastCte = cte[0];
      this.packets += 1;
      this.lastSubmit = performance.now();
    }

    getSteering() {
      return this.steer;
    }

    getSpeed() {
      return this.speed;
    }

    setSpeed(value) {
      if (Number.isFinite(value)) {
        this.speed = value;
      }
    }

    getPackets() {
      return this.packets;
    }

    getLastCte() {
      return this.lastCte;
    }
  }

  const query = new URLSearchParams(window.location.search);
  const querySpeed = Number.parseFloat(query.get("speed"));
  const speed = Number.isFinite(querySpeed) ? querySpeed : 15.0;

  window.MPCController = new LaneController(speed);
  console.log("Browser MPC controller ready (speed " + speed.toFixed(2) + " m/s)");
})();
