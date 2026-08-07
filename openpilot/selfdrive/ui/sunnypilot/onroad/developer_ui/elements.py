
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached


@dataclass
class UiElement:
  value: str
  label: str
  unit: str
  color: rl.Color
  val_text: str = ""
  label_text: str = ""
  unit_text: str = ""
  val_width: float = 0.0
  label_width: float = 0.0
  unit_width: float = 0.0
  total_width: float = 0.0

  def measure(self, font, font_size: int):
    self.label_text = f"{self.label} "
    self.val_text = self.value
    self.unit_text = f" {self.unit}" if self.unit else ""

    self.label_width = measure_text_cached(font, self.label_text, font_size, 0).x
    self.val_width = measure_text_cached(font, self.val_text, font_size, 0).x
    self.unit_width = measure_text_cached(font, self.unit_text, font_size, 0).x if self.unit else 0

    self.total_width = self.label_width + self.val_width + self.unit_width


class LeadInfoElement:
  @staticmethod
  def get_lead_status(sm):
    lead_one = sm['radarState'].leadOne
    return lead_one.present, lead_one.dRel, lead_one.vRel

  @staticmethod
  def get_lead_color(lead_d_rel: float, lead_v_rel: float = 0.0, use_v_rel: bool = False) -> rl.Color:
    if use_v_rel:
      if lead_v_rel < -4.4704:
        return rl.RED
      elif lead_v_rel < 0:
        return rl.Color(255, 188, 0, 255)  # Orange
    else:
      if lead_d_rel < 5:
        return rl.RED
      elif lead_d_rel < 15:
        return rl.Color(255, 188, 0, 255)  # Orange
    return rl.WHITE


class LateralControlElement:
  @staticmethod
  def get_lat_color(lat_active: bool, steer_override: bool, angle_steers: float = 0.0,
                    check_angle: bool = False) -> rl.Color:
    color = rl.WHITE
    if lat_active:
      color = rl.Color(145, 155, 149, 255) if steer_override else rl.Color(0, 255, 0, 255)

    if check_angle and lat_active:
      if abs(angle_steers) > 180:
        color = rl.RED
      elif abs(angle_steers) > 90:
        color = rl.Color(255, 188, 0, 255)
      else:
        # Keep green/grey from above
        pass
    elif check_angle and not lat_active:
      if abs(angle_steers) > 180:
        color = rl.RED
      elif abs(angle_steers) > 90:
        color = rl.Color(255, 188, 0, 255)

    return color


class RelDistElement(LeadInfoElement):
  def __init__(self):
    self.unit = tr("m")

  def update(self, sm, is_metric: bool) -> UiElement:
    lead_status, lead_d_rel, _ = self.get_lead_status(sm)
    value = f"{lead_d_rel:.0f}" if lead_status else "-"
    color = self.get_lead_color(lead_d_rel) if lead_status else rl.WHITE
    return UiElement(value, tr("REL DIST"), self.unit, color)


class RelSpeedElement(LeadInfoElement):
  def __init__(self):
    self.unit = tr("km/h")

  def update(self, sm, is_metric: bool) -> UiElement:
    lead_status, _, lead_v_rel = self.get_lead_status(sm)

    self.unit = tr("km/h") if is_metric else tr("mph")

    conversion = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
    value = f"{lead_v_rel * conversion:.0f}" if lead_status else "-"
    color = self.get_lead_color(0, lead_v_rel, use_v_rel=True) if lead_status else rl.WHITE

    return UiElement(value, tr("REL SPEED"), self.unit, color)


class CpuTempElement:
  def __init__(self):
    self.unit = "°C"

  @staticmethod
  def _get_cpu_temperatures():
    temperatures = []

    for thermal_zone in __import__("glob").glob("/sys/class/thermal/thermal_zone*/temp"):
      try:
        zone_dir = __import__("os").path.dirname(thermal_zone)
        zone_type_path = __import__("os").path.join(zone_dir, "type")

        with open(zone_type_path, "r") as f:
          zone_type = f.read().strip().lower()

        # Only use actual CPU user-temperature zones on C3.
        # Ignore -step and -lowf thermal-management zones.
        if not (zone_type.startswith("cpu") and zone_type.endswith("-usr")):
          continue

        with open(thermal_zone, "r") as f:
          temp = int(f.read().strip()) / 1000.0

        if -20.0 < temp < 150.0:
          temperatures.append(temp)
      except (OSError, ValueError):
        continue

    return temperatures

  def update(self, sm, is_metric: bool) -> UiElement:
    temperatures = self._get_cpu_temperatures()

    if not temperatures:
      return UiElement("-", tr("CPU TEMP"), self.unit, rl.WHITE)

    max_temp = max(temperatures)
    value = f"{max_temp:.0f}"

    if max_temp >= 80.0:
      color = rl.RED
    elif max_temp >= 70.0:
      color = rl.Color(255, 188, 0, 255)  # Orange
    else:
      color = rl.Color(0, 255, 0, 255)  # Green

    return UiElement(value, tr("CPU TEMP"), self.unit, color)


class SteeringAngleElement(LateralControlElement):
  def __init__(self):
    self.unit = ""

  def update(self, sm, is_metric: bool) -> UiElement:
    car_state = sm['carState']
    angle_steers = car_state.steeringAngleDeg
    lat_active = sm['carControl'].latActive
    steer_override = car_state.steeringPressed

    value = f"{angle_steers:.1f}°"
    color = self.get_lat_color(lat_active, steer_override, angle_steers, check_angle=True)

    return UiElement(value, tr("REAL STEER"), self.unit, color)


class DesiredSteeringAngleElement(LateralControlElement):
  def __init__(self):
    self.unit = ""

  def update(self, sm, is_metric: bool) -> UiElement:
    car_state = sm['carState']
    controls_state = sm['controlsState']
    lat_active = sm['carControl'].latActive
    angle_steers = car_state.steeringAngleDeg
    steer_angle_desired = controls_state.lateralControlState.angleState.steeringAngleDeg

    value = f"{steer_angle_desired:.1f}°" if lat_active else "-"

    color = rl.WHITE
    if lat_active:
      if abs(angle_steers) > 180:
        color = rl.RED
      elif abs(angle_steers) > 90:
        color = rl.Color(255, 188, 0, 255)
      else:
        color = rl.Color(0, 255, 0, 255)

    return UiElement(value, tr("DESIRED STEER"), self.unit, color)


class ActualLateralAccelElement(LateralControlElement):
  def __init__(self):
    self.unit = tr("m/s^2")

  def update(self, sm, is_metric: bool) -> UiElement:
    controls_state = sm['controlsState']
    curvature = controls_state.curvature
    v_ego = sm['carState'].vEgo
    roll = sm['liveParameters'].roll if sm.valid['liveParameters'] else 0.0
    lat_active = sm['carControl'].latActive
    steer_override = sm['carState'].steeringPressed

    actual_lat_accel = (curvature * v_ego ** 2) - (roll * 9.81)
    value = f"{actual_lat_accel:.2f}"
    color = self.get_lat_color(lat_active, steer_override)

    return UiElement(value, tr("ACTUAL L.A."), self.unit, color)


class DesiredLateralAccelElement(LateralControlElement):
  def __init__(self):
    self.unit = tr("m/s^2")

  def update(self, sm, is_metric: bool) -> UiElement:
    controls_state = sm['controlsState']
    desired_curvature = controls_state.desiredCurvature
    v_ego = sm['carState'].vEgo
    roll = sm['liveParameters'].roll if sm.valid['liveParameters'] else 0.0
    lat_active = sm['carControl'].latActive
    steer_override = sm['carState'].steeringPressed

    desired_lat_accel = (desired_curvature * v_ego ** 2) - (roll * 9.81)
    value = f"{desired_lat_accel:.2f}" if lat_active else "-"
    color = self.get_lat_color(lat_active, steer_override)

    return UiElement(value, tr("DESIRED L.A."), self.unit, color)


class DesiredSteeringPIDElement(LateralControlElement):
  def __init__(self):
    self.unit = ""

  def update(self, sm, is_metric: bool) -> UiElement:
    car_state = sm['carState']
    controls_state = sm['controlsState']
    lat_active = sm['carControl'].latActive
    angle_steers = car_state.steeringAngleDeg
    steer_angle_desired = controls_state.lateralControlState.pidState.steeringAngleDesiredDeg

    value = f"{steer_angle_desired:.1f}°" if lat_active else "-"

    color = rl.WHITE
    if lat_active:
      if abs(angle_steers) > 180:
        color = rl.RED
      elif abs(angle_steers) > 90:
        color = rl.Color(255, 188, 0, 255)
      else:
        color = rl.Color(0, 255, 0, 255)

    return UiElement(value, tr("DESIRED STEER"), self.unit, color)


class AEgoElement:
  def __init__(self):
