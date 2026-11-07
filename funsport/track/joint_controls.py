"""Opt-in public-statistics controls for synthetic joint track generation."""
from dataclasses import dataclass
import bisect
import math

from .geom import (ring_point_at, to_bd, meters_per_deg_lat, meters_per_deg_lng)
from ..coordinates import bd09_to_gcj02, distance_m


@dataclass(frozen=True)
class JointControls:
    """Explicit bounded controls; no coordinate or per-fix training data accepted.

    speed_sigma: multiplicative log-speed residual standard deviation (0..0.25).
    speed_tau_s: correlation time in seconds (1..600).
    start_transient: start speed multiplier amplitude (0..0.25).
    finish_transient: finish speed multiplier amplitude (0..0.25).
    cadence_slope_spm_per_mps: cadence response to smoothed speed (0..60).
    cadence_tau_s: cadence response time constant (1..300).
    cadence_residual_spm: independent cadence residual standard deviation (0..12).
    lateral_mean_m: signed side offset mean (−8..8 m).
    lateral_amplitude_m: smooth bounded side offset amplitude (0..8 m).
    lateral_tau_s: spatial scale in metres (5..2000); name retained for compatibility.
    The joint 8 m parameter cap is a model assumption, not a user hard limit.
    """
    speed_sigma: float = 0.0
    speed_tau_s: float = 60.0
    start_transient: float = 0.0
    finish_transient: float = 0.0
    cadence_slope_spm_per_mps: float = 0.0
    cadence_tau_s: float = 15.0
    cadence_residual_spm: float = 0.0
    lateral_mean_m: float = 0.0
    lateral_amplitude_m: float = 0.0
    lateral_tau_s: float = 100.0

    def validate(self):
        bounds = {
            "speed_sigma": (0.0, 0.25), "speed_tau_s": (1.0, 600.0),
            "start_transient": (0.0, 0.25), "finish_transient": (0.0, 0.25),
            "cadence_slope_spm_per_mps": (0.0, 60.0),
            "cadence_tau_s": (1.0, 300.0), "cadence_residual_spm": (0.0, 12.0),
            "lateral_mean_m": (-8.0, 8.0), "lateral_amplitude_m": (0.0, 8.0),
            "lateral_tau_s": (5.0, 2000.0),
        }
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"invalid joint control: {name}")
        if abs(self.lateral_mean_m) + self.lateral_amplitude_m > 8.0:
            raise ValueError("invalid joint control: lateral offset bound")
        return self


class LateralField:
    """Continuous progress field; the source polyline is never densified/remeasured.

    Tangent angle is the 10 m box average of unwrapped source-segment headings,
    extended constantly at endpoints. Exact 180-degree reversals turn left.
    This frame is continuous at corners/revisits and invariant to collinear
    subdivision (up to source projection roundoff). Positive normal is left.
    """
    FRAME_RADIUS_M = 5.0
    QUANTIZATION_TOLERANCE_M = 1e-8

    def __init__(self, dense, arcs, controls):
        self.dense, self.arcs, self.controls = dense, arcs, controls
        self.bound = abs(controls.lateral_mean_m) + controls.lateral_amplitude_m
        self.starts, self.angles, self.integrals = [], [], [0.0]
        for i in range(len(dense) - 1):
            length = arcs[i + 1] - arcs[i]
            if length <= 0:
                continue
            angle = math.atan2(dense[i + 1][1] - dense[i][1],
                               dense[i + 1][0] - dense[i][0])
            if self.angles:
                turn = (angle - self.angles[-1] + math.pi) % math.tau - math.pi
                if abs(turn + math.pi) < 1e-10:
                    turn = math.pi
                angle = self.angles[-1] + turn
            self.starts.append(arcs[i])
            self.angles.append(angle)
            self.integrals.append(self.integrals[-1] + angle * length)
        if not self.angles:
            raise ValueError("Lateral field requires positive source road length")
        # Derivative bound for the continuous field before grid-dependent contraction.
        c = controls
        self.lipschitz = (1.0 + (abs(c.lateral_mean_m) + math.tau * c.lateral_amplitude_m) /
                          c.lateral_tau_s + self.bound *
                          (max(self.angles) - min(self.angles)) / (2 * self.FRAME_RADIUS_M))

    def _angle_integral(self, s):
        if s <= 0:
            return s * self.angles[0]
        if s >= self.arcs[-1]:
            return self.integrals[-1] + (s - self.arcs[-1]) * self.angles[-1]
        i = bisect.bisect_right(self.starts, s) - 1
        return self.integrals[i] + (s - self.starts[i]) * self.angles[i]

    def frame(self, s):
        h = self.FRAME_RADIUS_M
        angle = (self._angle_integral(s + h) - self._angle_integral(s - h)) / (2 * h)
        tangent = (math.cos(angle), math.sin(angle))
        return tangent, (-tangent[1], tangent[0])

    def point(self, s):
        x, y = ring_point_at(self.dense, self.arcs, s, open_path=True)
        c = self.controls
        offset = (c.lateral_mean_m * -math.expm1(-s / c.lateral_tau_s) +
                  c.lateral_amplitude_m * math.sin(math.tau * s / c.lateral_tau_s))
        _, normal = self.frame(s)
        return x + normal[0] * offset, y + normal[1] * offset

    def sample(self, s, jitter, center):
        """Solve the offset inside each candidate, before chord/road acceptance.

        Bound final six-decimal BD distance from the unquantized source at the
        SAME progress: local Euclidean, BD Haversine and converted-GCJ Haversine.
        No positive physical tolerance is added for jitter/quantization. A grid
        half-diagonal reserve is a contraction, never a relaxed amplitude bound.
        Tiny envelopes with no feasible quantized candidate fail explicitly.
        """
        source = ring_point_at(self.dense, self.arcs, s, open_path=True)
        bx, by = self.point(s)
        dx, dy = bx + jitter[0] - source[0], by + jitter[1] - source[1]
        source_bd = to_bd(*source, *center)
        source_gcj = bd09_to_gcj02(*source_bd)
        grid_radius = 0.5e-6 * math.hypot(meters_per_deg_lat(center[0]),
                                        meters_per_deg_lng(center[0]))
        radius = self.bound
        for _ in range(8):
            norm = math.hypot(dx, dy)
            scale = min(1.0, radius / norm) if norm else 1.0
            px, py = source[0] + dx * scale, source[1] + dy * scale
            bd = tuple(float(f"{v:.6f}") for v in to_bd(px, py, *center))
            local = ((bd[1] - source_bd[1]) * meters_per_deg_lng(center[0]),
                     (bd[0] - source_bd[0]) * meters_per_deg_lat(center[0]))
            gcj = bd09_to_gcj02(*bd)
            used = max(math.hypot(*local), distance_m(bd, source_bd), distance_m(gcj, source_gcj))
            if used <= self.bound + self.QUANTIZATION_TOLERANCE_M:
                return bx, by, px, py, gcj
            radius = max(0.0, radius - (used - self.bound) - grid_radius)
        raise ValueError("Lateral envelope has no feasible quantized candidate")

    def brackets(self, start, end, center):
        """Bounded numerical forward search of a nonlinear candidate field.

        Evaluations are not route vertices or output fixes. The initial continuous
        field moves at most half a BD grid width per step; grid-dependent inward
        contractions may jump. Source vertices are visited too. This finite scan
        can miss feasible cells and is not a global feasibility certificate.
        """
        grid = 1e-6 * min(meters_per_deg_lat(center[0]), meters_per_deg_lng(center[0]))
        step = min(0.025, grid / (2 * self.lipschitz))
        low = start
        for stop in [s for s in self.arcs[bisect.bisect_right(self.arcs, start):] if s < end] + [end]:
            count = max(1, math.ceil((stop - low) / step))
            for i in range(1, count + 1):
                yield low + (stop - low) * i / count
            low = stop


def offset_route(dense, arcs, controls):
    """Vertex-only diagnostic view; generation evaluates LateralField.point(s)."""
    if controls.lateral_mean_m == 0 and controls.lateral_amplitude_m == 0:
        return dense
    field = LateralField(dense, arcs, controls)
    return [field.point(s) for s in arcs]


def controls_from_training_stats(stats, base=None):
    """Fit from aggregate statistics only; coordinate/per-fix inputs are rejected.

    Supported keys map speed log spread/correlation, cadence-speed slope/residual,
    transients, and lateral offset mean/amplitude/scale. Missing keys retain base.
    """
    if not isinstance(stats, dict) or any(not isinstance(k, str) for k in stats):
        raise ValueError("training statistics must be a scalar-key mapping")
    mapping = {
        "speed_log_sigma": "speed_sigma", "speed_sigma": "speed_sigma",
        "speed_correlation_s": "speed_tau_s", "speed_tau_s": "speed_tau_s",
        "cadence_speed_slope": "cadence_slope_spm_per_mps",
        "cadence_slope_spm_per_mps": "cadence_slope_spm_per_mps",
        "cadence_residual_spm": "cadence_residual_spm",
        "start_transient": "start_transient", "finish_transient": "finish_transient",
        "cadence_tau_s": "cadence_tau_s",
        "lateral_mean_m": "lateral_mean_m",
        "lateral_amplitude_m": "lateral_amplitude_m",
        "lateral_tau_s": "lateral_tau_s",
    }
    params = dict(vars((base or JointControls()).validate()))
    for source, value in stats.items():
        if source not in mapping:
            raise ValueError(f"unknown aggregate training statistic: {source}")
        target = mapping[source]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"invalid aggregate training statistic: {source}")
        params[target] = value
    return JointControls(**params).validate()
