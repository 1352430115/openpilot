import math
import os
from pathlib import Path
import numpy as np
from collections import deque
from datetime import datetime, timedelta, timezone

from cereal import log
from opendbc.car.lateral import get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 1.0
KI = 0.3
KD = 0.0
INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
FRICTION_THRESHOLD = 0.3

VERSION = 0

# ===== Lateral Debug Logger V4 =====
TW_TZ = timezone(timedelta(hours=8))
LATERAL_LOG_DIR = "/data/media/0/realdata/lateral_logs"

# Curve detection thresholds
CURVE_ENTRY_CURVATURE   = 0.0015   # |desired_curvature| must exceed this to start a curve
CURVE_EXIT_CURVATURE    = 0.0008   # |desired_curvature| must drop below this to end a curve
CURVE_MIN_SPEED_MS      = 10.0     # m/s — ignore low-speed events
CURVE_MIN_DURATION_S    = 1.0      # seconds — discard very short blips
OVERSHOOT_THRESHOLD_PCT = 8.0      # % overshoot relative to peak desired to flag event
LOG_INTERVAL_S          = 0.1      # sample every 0.1 s


# ---------------------------------------------------------------------------
# Helper dataclass (plain dict-style, no dataclasses import needed)
# ---------------------------------------------------------------------------
def _empty_curve():
  return {
    "id": 0,
    "direction": "",          # "LEFT" or "RIGHT"
    "start_time": None,
    "end_time": None,
    # per-sample accumulators
    "samples": [],            # list of snapshot dicts (0.1 s cadence)
    # overshoot tracking
    "overshoot_events": [],   # list of overshoot dicts
    "_in_overshoot": False,
    "_overshoot_start": None,
    "_overshoot_peak_desired": 0.0,
    "_overshoot_peak_measured": 0.0,
    "_overshoot_peak_pct": 0.0,
  }


# ---------------------------------------------------------------------------
class CurveAnalyser:
  """
  Manages curve lifecycle: entry → sampling → exit → summary + diagnosis.
  Called from LatControlTorque at every active frame (100 Hz).
  Writes to log file at 0.1 s cadence while in a curve.
  Writes full per-curve summary + diagnosis when curve ends.
  Accumulates session statistics for LEFT vs RIGHT comparison.
  """

  def __init__(self, log_dir: str):
    self._log_dir = log_dir
    os.makedirs(log_dir, exist_ok=True)

    self._curve_id = 0
    self._in_curve = False
    self._curve = _empty_curve()

    # 0.1-s sample gate
    self._last_sample_time: float = 0.0   # wall-clock seconds (float epoch)

    # Session-level LEFT / RIGHT stats
    self._session: dict = {
      "LEFT":  self._empty_session_side(),
      "RIGHT": self._empty_session_side(),
    }

  @staticmethod
  def _empty_session_side():
    return {
      "count": 0,
      "total_curv_error": 0.0,
      "max_curv_error": 0.0,
      "total_max_curv_error": 0.0,
      "overshoot_count": 0,
      "saturated_curves": 0,
      "steer_limited_curves": 0,
      "freeze_i_curves": 0,
    }

  # ------------------------------------------------------------------
  def update(self, now: datetime, desired_curvature: float,
             measured_curvature: float, speed_ms: float,
             pid_p: float, pid_i: float, pid_f: float,
             output_torque: float, ff: float,
             saturated: bool, steer_limited: bool,
             freeze_integrator: bool, curvature_limited: bool,
             roll_compensation: float, measurement_rate: float,
             setpoint: float, measurement: float,
             steer_angle_deg: float, steering_rate_deg: float, roll: float):
    """Called every control frame (≈100 Hz). Manages curve state machine."""

    abs_desired = abs(desired_curvature)
    direction = "RIGHT" if desired_curvature < 0 else "LEFT"
    now_ts = now.timestamp()

    # ── 1. Curve entry ───────────────────────────────────────────────
    if not self._in_curve:
      if abs_desired > CURVE_ENTRY_CURVATURE and speed_ms > CURVE_MIN_SPEED_MS:
        self._in_curve = True
        self._curve_id += 1
        self._curve = _empty_curve()
        self._curve["id"] = self._curve_id
        self._curve["direction"] = direction
        self._curve["start_time"] = now
        self._last_sample_time = 0.0   # force immediate first sample
      else:
        return   # straight — nothing to do

    # ── 2. Direction flip mid-curve → force close + re-open ──────────
    if self._in_curve and self._curve["direction"] != direction and abs_desired > CURVE_ENTRY_CURVATURE:
      self._close_curve(now)
      self._in_curve = True
      self._curve_id += 1
      self._curve = _empty_curve()
      self._curve["id"] = self._curve_id
      self._curve["direction"] = direction
      self._curve["start_time"] = now
      self._last_sample_time = 0.0

    # ── 3. Overshoot detection (runs every frame for resolution) ──────
    self._update_overshoot(now, desired_curvature, measured_curvature)

    # ── 4. 0.1-s sample ──────────────────────────────────────────────
    if now_ts - self._last_sample_time >= LOG_INTERVAL_S:
      self._last_sample_time = now_ts
      curv_err = abs(desired_curvature - measured_curvature)
      snap = {
        "ts": now,
        "speed_kph": speed_ms * 3.6,
        "steer_deg": steer_angle_deg,
        "steer_rate": steering_rate_deg,
        "roll": roll,
        "roll_comp": roll_compensation,
        "desired_curv": desired_curvature,
        "measured_curv": measured_curvature,
        "curv_err": curv_err,
        "setpoint": setpoint,
        "measurement": measurement,
        "lat_err": setpoint - measurement,
        "ff": ff,
        "pid_p": pid_p,
        "pid_i": pid_i,
        "pid_f": pid_f,
        "torque": output_torque,
        "meas_rate": measurement_rate,
        "freeze_i": freeze_integrator,
        "saturated": saturated,
        "steer_limited": steer_limited,
        "curv_limited": curvature_limited,
      }
      self._curve["samples"].append(snap)
      self._write_raw_line(snap)

    # ── 5. Curve exit ─────────────────────────────────────────────────
    if max(abs(desired_curvature), abs(measured_curvature)) < CURVE_EXIT_CURVATURE:
      self._close_curve(now)

  # ------------------------------------------------------------------
  def _update_overshoot(self, now: datetime, desired: float, measured: float):
    """
    Overshoot = measured curvature exceeds desired (same sign) by >THRESHOLD %.
    Uses signed comparison so left/right are handled correctly.
    """
    c = self._curve
    # Both must have the same sign (same turn direction); use desired as reference.
    if desired == 0.0:
      return
    # Overshoot when measured exceeds desired in magnitude (same direction)
    same_sign = (desired * measured) > 0
    if not same_sign:
      return

    pct = (abs(measured) - abs(desired)) / abs(desired) * 100.0

    if pct > OVERSHOOT_THRESHOLD_PCT:
      if not c["_in_overshoot"]:
        c["_in_overshoot"] = True
        c["_overshoot_start"] = now
        c["_overshoot_peak_desired"] = desired
        c["_overshoot_peak_measured"] = measured
        c["_overshoot_peak_pct"] = pct
      else:
        if pct > c["_overshoot_peak_pct"]:
          c["_overshoot_peak_desired"] = desired
          c["_overshoot_peak_measured"] = measured
          c["_overshoot_peak_pct"] = pct
    else:
      if c["_in_overshoot"]:
        # Overshoot just ended
        duration = (now - c["_overshoot_start"]).total_seconds()
        c["overshoot_events"].append({
          "start": c["_overshoot_start"],
          "end": now,
          "duration_s": duration,
          "peak_desired": c["_overshoot_peak_desired"],
          "peak_measured": c["_overshoot_peak_measured"],
          "peak_pct": c["_overshoot_peak_pct"],
        })
        c["_in_overshoot"] = False

  # ------------------------------------------------------------------
  def _close_curve(self, now: datetime):
    """Finalise open overshoot, compute summary, write it, update session stats."""
    c = self._curve
    # Close any open overshoot
    if c["_in_overshoot"]:
      duration = (now - c["_overshoot_start"]).total_seconds()
      c["overshoot_events"].append({
        "start": c["_overshoot_start"],
        "end": now,
        "duration_s": duration,
        "peak_desired": c["_overshoot_peak_desired"],
        "peak_measured": c["_overshoot_peak_measured"],
        "peak_pct": c["_overshoot_peak_pct"],
      })
      c["_in_overshoot"] = False

    c["end_time"] = now
    duration_s = (c["end_time"] - c["start_time"]).total_seconds() if c["start_time"] else 0.0
    self._in_curve = False

    if duration_s < CURVE_MIN_DURATION_S or len(c["samples"]) == 0:
      return   # too short — discard silently

    self._write_curve_summary(c, duration_s)
    self._update_session(c)
    total = self._session["LEFT"]["count"] + self._session["RIGHT"]["count"]
    if total > 0 and total % 10 == 0:
      self.write_session_comparison()

  # ------------------------------------------------------------------
  def _compute_stats(self, samples: list, key: str):
    vals = [s[key] for s in samples]
    if not vals:
      return 0.0, 0.0
    return max(vals), sum(vals) / len(vals)

  # ------------------------------------------------------------------
  def _write_raw_line(self, snap: dict):
    """Append one 0.1-s sample line to today's raw log."""
    try:
      ts: datetime = snap["ts"]
      fn = os.path.join(self._log_dir, ts.strftime("%Y-%m-%d_lateral.log"))
      direction = "RIGHT" if snap["desired_curv"] < 0 else "LEFT"
      line = (
        f"{ts.strftime('%H:%M:%S.%f')[:-3]},"
        f"curve={self._curve['id']},"
        f"{direction},"
        f"speed={snap['speed_kph']:.1f},"
        f"steer={snap['steer_deg']:.2f},"
        f"steerRate={snap['steer_rate']:.2f},"
        f"roll={snap['roll']:.5f},"
        f"rollComp={snap['roll_comp']:.4f},"
        f"desiredCurv={snap['desired_curv']:.6f},"
        f"measuredCurv={snap['measured_curv']:.6f},"
        f"curvErr={snap['curv_err']:.6f},"
        f"desiredLat={snap['setpoint']:.4f},"
        f"actualLat={snap['measurement']:.4f},"
        f"latErr={snap['lat_err']:.4f},"
        f"ff={snap['ff']:.4f},"
        f"P={snap['pid_p']:.4f},I={snap['pid_i']:.4f},F={snap['pid_f']:.4f},"
        f"torque={snap['torque']:.4f},"
        f"measRate={snap['meas_rate']:.4f},"
        f"freezeI={snap['freeze_i']},"
        f"saturated={snap['saturated']},"
        f"steerLimited={snap['steer_limited']},"
        f"curvLimited={snap['curv_limited']}\n"
      )
      with open(fn, "a") as f:
        f.write(line)
    except Exception:
      pass

  # ------------------------------------------------------------------
  def _write_curve_summary(self, c: dict, duration_s: float):
    """Write the per-curve summary block to the summary log file."""
    try:
      samples = c["samples"]
      start: datetime = c["start_time"]
      end: datetime   = c["end_time"]
      direction       = c["direction"]
      curve_id        = c["id"]

      # ── Compute statistics ────────────────────────────────────────
      max_desired_curv = max(abs(s["desired_curv"]) for s in samples)
      max_measured_curv = max(abs(s["measured_curv"]) for s in samples)

      curv_errors = [s["curv_err"] for s in samples]
      max_curv_err = max(curv_errors)
      avg_curv_err = sum(curv_errors) / len(curv_errors)

      lat_errors = [abs(s["lat_err"]) for s in samples]
      max_lat_err = max(lat_errors)
      avg_lat_err = sum(lat_errors) / len(lat_errors)

      torques = [abs(s["torque"]) for s in samples]
      max_torque = max(torques)
      avg_torque = sum(torques) / len(torques)

      max_p = max(abs(s["pid_p"]) for s in samples)
      max_i = max(abs(s["pid_i"]) for s in samples)
      max_f = max(abs(s["pid_f"]) for s in samples)

      speeds = [s["speed_kph"] for s in samples]
      avg_speed = sum(speeds) / len(speeds)
      max_speed = max(speeds)

      sat_count   = sum(1 for s in samples if s["saturated"])
      sl_count    = sum(1 for s in samples if s["steer_limited"])
      fi_count    = sum(1 for s in samples if s["freeze_i"])
      n = len(samples)

      any_saturated    = sat_count > 0
      any_steer_limited = sl_count > 0
      any_freeze_i     = fi_count > 0

      # Overshoot
      overshoot_events = c["overshoot_events"]
      had_overshoot = len(overshoot_events) > 0

      # Max curvature error % relative to peak desired
      curv_err_pct = (max_curv_err / max_desired_curv * 100.0) if max_desired_curv > 0 else 0.0

      # ── Diagnosis ─────────────────────────────────────────────────
      diagnosis, suggestions = self._diagnose(
        direction=direction,
        max_curv_err=max_curv_err,
        avg_curv_err=avg_curv_err,
        curv_err_pct=curv_err_pct,
        max_desired_curv=max_desired_curv,
        max_measured_curv=max_measured_curv,
        any_saturated=any_saturated,
        any_steer_limited=any_steer_limited,
        any_freeze_i=any_freeze_i,
        had_overshoot=had_overshoot,
        overshoot_events=overshoot_events,
        sat_pct=sat_count / n * 100,
        fi_pct=fi_count / n * 100,
        max_i=max_i,
        max_torque=max_torque,
        avg_torque=avg_torque,
      )

      # ── Format summary block ──────────────────────────────────────
      sep  = "=" * 60
      sep2 = "-" * 40
      lines = [
        "",
        sep,
        f"  CURVE {curve_id:04d} SUMMARY  [{direction}]",
        sep,
        f"  Time      : {start.strftime('%H:%M:%S.%f')[:-3]} → {end.strftime('%H:%M:%S.%f')[:-3]}",
        f"  Duration  : {duration_s:.2f} s",
        f"  Speed     : avg {avg_speed:.1f} km/h  max {max_speed:.1f} km/h",
        sep2,
        "  CURVATURE",
        f"    Max Desired   : {max_desired_curv:.6f}",
        f"    Max Measured  : {max_measured_curv:.6f}",
        f"    Max Error     : {max_curv_err:.6f}  ({curv_err_pct:.1f}%)",
        f"    Avg Error     : {avg_curv_err:.6f}",
        sep2,
        "  LAT ACCEL ERROR",
        f"    Max Error     : {max_lat_err:.4f} m/s²",
        f"    Avg Error     : {avg_lat_err:.4f} m/s²",
        sep2,
        "  PID / TORQUE",
        f"    P max         : {max_p:.4f}",
        f"    I max         : {max_i:.4f}",
        f"    F max         : {max_f:.4f}",
        f"    Torque max    : {max_torque:.4f}",
        f"    Torque avg    : {avg_torque:.4f}",
        sep2,
        "  FLAGS",
        f"    Saturated     : {'YES (' + str(sat_count) + '/' + str(n) + ' samples)' if any_saturated else 'NO'}",
        f"    SteerLimited  : {'YES (' + str(sl_count) + '/' + str(n) + ' samples)' if any_steer_limited else 'NO'}",
        f"    FreezeI       : {'YES (' + str(fi_count) + '/' + str(n) + ' samples)' if any_freeze_i else 'NO'}",
      ]

      # Overshoot block
      lines += [sep2, "  OVERSHOOT"]
      if had_overshoot:
        for idx, ov in enumerate(overshoot_events, 1):
          lines += [
            f"    Event {idx}:",
            f"      Start    : {ov['start'].strftime('%H:%M:%S.%f')[:-3]}",
            f"      Duration : {ov['duration_s']:.2f} s",
            f"      Desired  : {ov['peak_desired']:.6f}",
            f"      Measured : {ov['peak_measured']:.6f}",
            f"      Overshoot: +{ov['peak_pct']:.1f}%",
          ]
      else:
        lines.append("    NONE")

      # Diagnosis block
      lines += [sep2, "  DIAGNOSIS"]
      for flag, status, note in diagnosis:
        mark = "✓" if status else "✗"
        lines.append(f"    {mark} {flag:<28s}  {note}")

      lines += [sep2, "  SUGGESTIONS"]
      if suggestions:
        for s in suggestions:
          lines.append(f"    • {s}")
      else:
        lines.append("    None — controller behaviour appears normal for this curve.")

      lines += [sep, ""]

      # ── Write to summary log ──────────────────────────────────────
      fn = os.path.join(
        self._log_dir,
        start.strftime("%Y-%m-%d_lateral_summary.log")
      )
      with open(fn, "a") as f:
        f.write("\n".join(lines) + "\n")

    except Exception:
      pass

  # ------------------------------------------------------------------
  def _diagnose(self, direction, max_curv_err, avg_curv_err, curv_err_pct,
                max_desired_curv, max_measured_curv,
                any_saturated, any_steer_limited, any_freeze_i,
                had_overshoot, overshoot_events,
                sat_pct, fi_pct,
                max_i, max_torque, avg_torque):
    """
    Return (diagnosis list, suggestions list).
    diagnosis: list of (label, bool_present, note_str)
    suggestions: list of str
    """
    diag = []
    sugg = []

    # ── Planner curvature too large ───────────────────────────────
    planner_large = curv_err_pct > 15.0 and not any_saturated
    diag.append((
      "Planner curvature too large",
      planner_large,
      f"curvErr={curv_err_pct:.1f}% of desired" if planner_large else "within tolerance"
    ))
    if planner_large:
      sugg.append("Planner is over-requesting curvature. "
                  "Check PATH_OFFSET or lateral planner gain — "
                  "do NOT adjust PID first.")

    # ── Torque controller under-delivering ────────────────────────
    torque_low = (max_measured_curv < max_desired_curv * 0.80
                  and not any_saturated
                  and not any_steer_limited)
    diag.append((
      "Torque controller under-delivering",
      torque_low,
      "measured < 80% of desired" if torque_low else "output adequate"
    ))
    if torque_low:
      sugg.append("Torque output is insufficient. "
                  "Consider increasing latAccelFactor or KP at this speed range.")

    # ── Torque controller over-correcting ────────────────────────
    torque_over = had_overshoot and not planner_large
    diag.append((
      "Torque controller over-correcting",
      torque_over,
      "overshoot present without planner cause" if torque_over else "no overshoot from controller"
    ))
    if torque_over:
      sugg.append("Controller is over-shooting. "
                  "Consider reducing KP or increasing STEER_DELTA_DOWN to slow wind-down.")

    # ── EPS saturation ────────────────────────────────────────────
    diag.append((
      "EPS saturation",
      any_saturated,
      f"{sat_pct:.0f}% of samples" if any_saturated else "not saturated"
    ))
    if any_saturated:
      sugg.append("Steering actuator saturated. "
                  "The requested curvature exceeds hardware capability at this speed.")

    # ── Steering limited by safety ────────────────────────────────
    diag.append((
      "SteerLimited by safety",
      any_steer_limited,
      "safety limiter active" if any_steer_limited else "not limited"
    ))
    if any_steer_limited:
      sugg.append("Safety steer limiter triggered — integrator was frozen. "
                  "Check if STEER_MAX or safety envelope needs adjustment.")

    # ── Integrator accumulation ───────────────────────────────────
    i_high = max_i > 0.5
    diag.append((
      "PID integrator accumulation",
      i_high,
      f"max I={max_i:.3f}" if i_high else f"I={max_i:.3f} (normal)"
    ))
    if i_high:
      sugg.append("Large integrator term detected. "
                  "Verify FreezeI logic is working correctly and latAccelOffset is well-calibrated.")

    # ── FreezeI active too long ───────────────────────────────────
    fi_excess = fi_pct > 30.0
    diag.append((
      "FreezeI active >30% of curve",
      fi_excess,
      f"{fi_pct:.0f}% of samples frozen" if fi_excess else f"{fi_pct:.0f}% (acceptable)"
    ))
    if fi_excess:
      sugg.append("Integrator was frozen for most of the curve. "
                  "This prevents the I-term from converging — check steeringPressed or safety conditions.")

    return diag, sugg

  # ------------------------------------------------------------------
  def write_session_comparison(self):
    """
    Write LEFT vs RIGHT session comparison to summary log.
    Called externally (e.g. on process shutdown) — not used in hot path.
    """
    try:
      now = datetime.now(TW_TZ)
      fn = os.path.join(self._log_dir, now.strftime("%Y-%m-%d_lateral_summary.log"))
      L = self._session["LEFT"]
      R = self._session["RIGHT"]

      def safe_div(a, b):
        return a / b if b else 0.0

      lines = [
        "",
        "=" * 60,
        "  SESSION COMPARISON: LEFT vs RIGHT",
        "=" * 60,
        f"  {'Metric':<30s}  {'LEFT':>10s}  {'RIGHT':>10s}",
        "-" * 60,
        f"  {'Curve count':<30s}  {L['count']:>10d}  {R['count']:>10d}",
        f"  {'Avg curv error (avg of max)':<30s}  {safe_div(L['total_max_curv_error'], L['count']):>10.6f}  {safe_div(R['total_max_curv_error'], R['count']):>10.6f}",
        f"  {'Overall avg curv error':<30s}  {safe_div(L['total_curv_error'], L['count']):>10.6f}  {safe_div(R['total_curv_error'], R['count']):>10.6f}",
        f"  {'All-time max curv error':<30s}  {L['max_curv_error']:>10.6f}  {R['max_curv_error']:>10.6f}",
        f"  {'Overshoot events':<30s}  {L['overshoot_count']:>10d}  {R['overshoot_count']:>10d}",
        f"  {'Curves with saturation':<30s}  {L['saturated_curves']:>10d}  {R['saturated_curves']:>10d}",
        f"  {'Curves with steer limit':<30s}  {L['steer_limited_curves']:>10d}  {R['steer_limited_curves']:>10d}",
        f"  {'Curves with FreezeI':<30s}  {L['freeze_i_curves']:>10d}  {R['freeze_i_curves']:>10d}",
      ]

      # Asymmetry flag
      if L["count"] > 0 and R["count"] > 0:
        ratio = safe_div(
          safe_div(R["total_max_curv_error"], R["count"]),
          safe_div(L["total_max_curv_error"], L["count"])
        )
        lines += [
          "-" * 60,
          f"  RIGHT avg max error / LEFT avg max error = {ratio:.2f}x",
        ]
        if ratio > 1.5:
          lines.append("  ⚠  RIGHT curves show significantly higher error — asymmetry confirmed.")
        elif ratio < 0.67:
          lines.append("  ⚠  LEFT curves show significantly higher error — unexpected asymmetry.")
        else:
          lines.append("  ✓  Error is roughly symmetric — not a consistent directional bias.")

      lines += ["=" * 60, ""]
      with open(fn, "a") as f:
        f.write("\n".join(lines) + "\n")
    except Exception:
      pass

  # ------------------------------------------------------------------
  def _update_session(self, c: dict):
    side = c["direction"]
    samples = c["samples"]
    if not samples:
      return
    s = self._session[side]
    s["count"] += 1
    curv_errors = [smp["curv_err"] for smp in samples]
    max_ce = max(curv_errors)
    avg_ce = sum(curv_errors) / len(curv_errors)
    s["total_max_curv_error"] += max_ce
    s["total_curv_error"] += avg_ce
    if max_ce > s["max_curv_error"]:
      s["max_curv_error"] = max_ce
    s["overshoot_count"] += len(c["overshoot_events"])
    if any(smp["saturated"] for smp in samples):
      s["saturated_curves"] += 1
    if any(smp["steer_limited"] for smp in samples):
      s["steer_limited_curves"] += 1
    if any(smp["freeze_i"] for smp in samples):
      s["freeze_i_curves"] += 1

  # ------------------------------------------------------------------
  def cleanup_old_logs(self, now: datetime):
    """Delete log files older than 3 days."""
    try:
      cutoff = now - timedelta(days=3)
      for p in Path(self._log_dir).glob("*_lateral*.log"):
        try:
          stem = p.stem.split("_lateral")[0]
          d = datetime.strptime(stem, "%Y-%m-%d").replace(tzinfo=TW_TZ)
          if d < cutoff:
            p.unlink()
        except Exception:
          pass
    except Exception:
      pass


# ===========================================================================
class LatControlTorque(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len,
                                          maxlen=self.lat_accel_request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

    self._curve_analyser = CurveAnalyser(LATERAL_LOG_DIR)
    self._cleanup_counter = 0   # rate-limit file cleanup (every ~1000 frames ≈ 10 s)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature,
             calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                               CS.vEgo, params.roll)
      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
      future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
      self.lat_accel_request_buffer.append(future_desired_lateral_accel)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / lat_delay

      measurement = measured_curvature * CS.vEgo ** 2
      measurement_rate = self.measurement_rate_filter.update(
        (measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = setpoint - measurement

      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      ff -= self.torque_params.latAccelOffset
      ff += get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error,
                                        -measurement_rate,
                                        feedforward=ff,
                                        speed=CS.vEgo,
                                        freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      pid_log, output_torque = self.extension.update(
        CS, VM, self.pid, params, ff, pid_log, setpoint, measurement,
        calibrated_pose, roll_compensation,
        future_desired_lateral_accel, measurement, lateral_accel_deadzone,
        gravity_adjusted_future_lateral_accel,
        desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(
        self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

      # ── V4 Curve Analyser ──────────────────────────────────────────
      if CS.vEgo > CURVE_MIN_SPEED_MS:
        now = datetime.now(TW_TZ)
        self._curve_analyser.update(
          now=now,
          desired_curvature=desired_curvature,
          measured_curvature=measured_curvature,
          speed_ms=CS.vEgo,
          pid_p=float(self.pid.p),
          pid_i=float(self.pid.i),
          pid_f=float(self.pid.f),
          output_torque=output_torque,
          ff=ff,
          saturated=pid_log.saturated,
          steer_limited=steer_limited_by_safety,
          freeze_integrator=freeze_integrator,
          curvature_limited=curvature_limited,
          roll_compensation=roll_compensation,
          measurement_rate=measurement_rate,
          setpoint=setpoint,
          measurement=measurement,
          steer_angle_deg=CS.steeringAngleDeg,
          steering_rate_deg=CS.steeringRateDeg,
          roll=params.roll,
        )
        # Periodic old-file cleanup (every ~1000 frames ≈ 10 s at 100 Hz)
        self._cleanup_counter += 1
        if self._cleanup_counter >= 1000:
          self._cleanup_counter = 0
          self._curve_analyser.cleanup_old_logs(now)

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
