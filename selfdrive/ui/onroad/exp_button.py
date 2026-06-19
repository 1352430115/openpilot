import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# DEC state values from capnp: acc=0, blended=1
_DEC_STATE_BLENDED = 1


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False
    self._engageable: bool = False

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    # DEC state cache - updated only when new longitudinalPlanSP message arrives
    self._dec_active: bool = False
    self._dec_is_blended: bool = False

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

    # Only update DEC state when a fresh longitudinalPlanSP message has arrived.
    # Reading a stale capnp object every frame caused flickering.
    try:
      if ui_state.sm.updated["longitudinalPlanSP"]:
        dec = ui_state.sm["longitudinalPlanSP"].dec
        self._dec_active = bool(dec.active)
        self._dec_is_blended = str(dec.state) == "blended"
    except Exception:
      pass

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)

      # Hold new state temporarily so the icon reflects the manual toggle
      # before DEC has a chance to update
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed or not self._engageable else 255

    texture = self._txt_exp if self._held_or_actual_mode() else self._txt_wheel
    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)
    rl.draw_texture_ex(texture, rl.Vector2(center_x - texture.width / 2, center_y - texture.height / 2), 0.0, 1.0, self._white_color)

  def _held_or_actual_mode(self) -> bool:
    now = time.monotonic()

    # During hold window, show the manually toggled state
    if self._hold_end_time is not None:
      if now < self._hold_end_time:
        return self._held_mode
      else:
        self._hold_end_time = None
        self._held_mode = None

    # When DEC is active, reflect its actual operating mode
    # acc=0 -> wheel icon (False), blended=1 -> flask icon (True)
    if self._dec_active:
      return self._dec_is_blended

    return self._experimental_mode

  def _is_toggle_allowed(self) -> bool:
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    return ui_state.has_longitudinal_control
