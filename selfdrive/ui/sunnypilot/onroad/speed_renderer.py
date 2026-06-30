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

# ==========================================================
# SCI V3.0（時速顏色指示器）
# Date : 2026-06-29
#
# 綠色：OP 正在加速（或人踩油門）
# 白色：滑行／維持速度（含引擎煞車）
# 紅色：ACC 主動液壓煞車（後車看到煞車燈亮）
#
# 煞車判斷來源：carState.brakeLightsDEPRECATED
#   目前使用 Toyota PCM_CRUISE 的 ACC_BRAKING 訊號。
#   此訊號代表 ACC 已開始主動減速，可能包含收油滑行，
#   並非車輛實際煞車燈硬體回報。
#   人踩煞車(brakePressed)仍維持最高優先權。
#
# 加速判斷來源：carState.gasPressed 或 carOutput.actuatorsOutput.accel > 0
#
# ※ 本功能僅影響 UI，不影響任何控制邏輯。
# ==========================================================

# 顏色定義（直接用 rl.Color，不依賴 COLORS 未定義的屬性）
_COLOR_ACCEL = rl.Color(0,   230, 100, 255)  # 綠：加速
_COLOR_BRAKE = rl.Color(255,  60,  60, 255)  # 紅：煞車
_COLOR_COAST = rl.WHITE                       # 白：滑行


class SpeedRenderer:
  def __init__(self):
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self.speed_color: rl.Color = _COLOR_COAST

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

  def update(self) -> None:
    sm = ui_state.sm
    car_state = sm['carState']

    # 速度計算（不動）
    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen and not ui_state.true_v_ego_ui else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    # 煞車：ACC_BRAKING + 人踩煞車
    # 人踩煞車(brakePressed)優先，ACC_BRAKING 用於 OP 主動減速顯示
    is_braking = car_state.brakePressed or car_state.brakeLightsDEPRECATED

    # 加速：人踩油門 或 ACC 輸出正加速
    is_accel = car_state.gasPressed or (not is_braking and sm['carOutput'].actuatorsOutput.accel > 0.0)

    if is_braking:
      self.speed_color = _COLOR_BRAKE
    elif is_accel:
      self.speed_color = _COLOR_ACCEL
    else:
      self.speed_color = _COLOR_COAST

  def render(self, rect: rl.Rectangle) -> None:
    if ui_state.hide_v_ego_ui:
      return

    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, self.speed_color)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
