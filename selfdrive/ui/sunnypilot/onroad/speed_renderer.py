"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.onroad.hud_renderer import FONT_SIZES, COLORS


class SpeedRenderer:
  def __init__(self):
    self.speed: float = 0.0
    self.speed_color = COLORS.WHITE
    self.v_ego_cluster_seen: bool = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

  def update(self) -> None:
    car_state = ui_state.sm['carState']
    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen and not ui_state.true_v_ego_ui else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    # ==========================================================
    # SCI V1（時速顏色指示器）
    # Version : 1.0
    # Date    : 2026-06-28
    #
    # 功能：
    # 依照 openpilot 最終輸出的縱向控制命令(actuators.accel)
    # 即時改變目前時速顏色。
    #
    # 綠色：OP 正在加速
    # 白色：滑行／維持速度
    # 紅色：OP 正在減速／煞車
    #
    # ※ 本功能僅影響 UI，不影響任何控制邏輯。
    # ==========================================================

    controls_state = ui_state.sm['controlsState']

    # ===== SCI V1 可調整參數 =====
    # 綠色門檻（m/s²）
    # 越小：越容易變綠
    # 越大：需要更大的加速才會變綠
    # 建議範圍：0.03 ~ 0.10
    SCI_ACCEL_GREEN = 0.05

    # 紅色門檻（m/s²）
    # 越接近 0（例如 -0.20）：越容易變紅
    # 越負（例如 -0.50）：需要更大的減速度才會變紅
    # 建議範圍：-0.20 ~ -0.50
    SCI_BRAKE_RED = -0.30

    # 讀取 Toyota CarController 最終輸出的縱向控制命令
    accel = controls_state.actuators.accel

    # OP 正在加速
    if accel > SCI_ACCEL_GREEN:
      self.speed_color = COLORS.GREEN
    # OP 正在減速／煞車
    elif accel < SCI_BRAKE_RED:
      self.speed_color = COLORS.RED
    # OP 滑行／維持速度
    else:
      self.speed_color = COLORS.WHITE


  def render(self, rect: rl.Rectangle) -> None:
    if ui_state.hide_v_ego_ui:
      return

    # Draw current speed and unit
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, self.speed_color)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
