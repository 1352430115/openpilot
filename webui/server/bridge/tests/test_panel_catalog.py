"""Contract tests for panel_catalog parameter exposure."""

from __future__ import annotations

import ast
import re
import unittest

from webui.server.bridge import carrot_tuning_api
from webui.server.bridge.panel_catalog import (CARROT_TUNING_UNAVAILABLE, SUBPANELS,
                                                    get_panel, panel_param_keys)

# Carrot tuning keys that are intentionally NOT rendered as user-tunable widgets:
# a cross-process diagnostic sink written by the carrot daemon, plus every param
# that no code in this tree consumes (see CARROT_TUNING_UNAVAILABLE for the reasons).
_PANEL_EXCLUDED = {"CarrotException"} | set(CARROT_TUNING_UNAVAILABLE)


def _config_nav_param_keys() -> set[str]:
  """Parse _DEFAULT_NAV_PARAMS keys from config.py without importing openpilot."""
  src = open(r"E:\sp\openpilot\sunnypilot\carrot\config.py", encoding="utf-8").read()
  for node in ast.walk(ast.parse(src)):
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) \
       and node.target.id == "_DEFAULT_NAV_PARAMS":
      return {k.value for k in node.value.keys}
    if isinstance(node, ast.Assign):
      for t in node.targets:
        if isinstance(t, ast.Name) and t.id == "_DEFAULT_NAV_PARAMS":
          return {k.value for k in node.value.keys}
  return set()


class PanelCatalogParamTests(unittest.TestCase):
  """Verify the P1 carrot params that are actually wired are exposed in the webui.

  VehicleSpeedCameraDistanceTime was in this list but no code reads it, so it is now
  hidden like the rest of the inert params - exposing it would advertise a control
  that cannot do anything. It stays registered, so re-adding it here is all that is
  needed once a reader exists.
  """

  _P1_PARAMS = {
    "VehicleNaviCanControl",
    "VehicleNaviSchoolZoneControl",
    "VehicleSpeedCameraControlMode",
    "AutoNaviSpeedBumpEndDistance",
    "LatSuspendAngleDeg",
  }

  # Cluster params are intentionally exposed: ClusterNaviMap* is read by
  # carrot_navi's ClusterNaviMapParamReader, and ClusterHud*/CarrotNaviHudMapProfile
  # are the first step of porting CarrotPilot's external-cluster subsystem - the
  # settings surface is complete and the values persist across sessions.
  _CLUSTER_EXPOSED = {
    "ClusterNaviMapTheme", "ClusterNaviMapType", "ClusterNaviMapFps",
    "ClusterHud", "ClusterHudTheme",
    "ClusterHudBrightness", "ClusterHudCameraViewMode", "ClusterHudCoreMode",
    "ClusterHudDebug", "ClusterHudEncoder", "ClusterHudLiveFps",
    "ClusterHudMirror", "ClusterHudOrientation", "ClusterHudPanelLayout",
    "ClusterHudPriority", "ClusterHudRadarDisplay", "ClusterHudRadarInfo",
    "ClusterHudRadarSourceColor", "ClusterHudScreenMode",
  }

  def test_carrot_tuning_exposes_p1_params(self):
    keys = set(panel_param_keys("navigation__carrot_tuning"))
    missing = self._P1_PARAMS - keys
    self.assertFalse(missing, f"navigation__carrot_tuning missing params: {sorted(missing)}")

  def test_carrot_tuning_exposes_cluster_params(self):
    """Cluster knobs are now first-class settings (part 1 of the cluster port)."""
    keys = set(panel_param_keys("navigation__carrot_tuning__display"))
    missing = self._CLUSTER_EXPOSED - keys
    self.assertFalse(missing, f"display panel missing cluster params: {sorted(missing)}")


class CarrotTuningFullCoverageTests(unittest.TestCase):
  """Every carrot tuning key must be backed by the API and the panel."""

  def test_all_config_keys_in_api_defaults(self):
    api_keys = set(carrot_tuning_api.CARROT_TUNING_DEFAULTS)
    cfg_keys = _config_nav_param_keys()
    missing = cfg_keys - api_keys
    self.assertFalse(missing, f"CARROT_TUNING_DEFAULTS missing keys: {sorted(missing)}")

  def test_all_config_keys_exposed_in_panel(self):
    panel_keys = set(panel_param_keys("navigation__carrot_tuning"))
    cfg_keys = _config_nav_param_keys()
    missing = cfg_keys - panel_keys - _PANEL_EXCLUDED
    self.assertFalse(missing, f"panel missing carrot params: {sorted(missing)}")

  def test_panel_params_backed_by_api(self):
    api_keys = set(carrot_tuning_api.CARROT_TUNING_DEFAULTS)
    panel_keys = set(panel_param_keys("navigation__carrot_tuning"))
    # CarrotEnabled is a master toggle dependency, not a carrot tuning key.
    orphan = panel_keys - api_keys - {"CarrotEnabled"}
    self.assertFalse(orphan, f"panel params not in API: {sorted(orphan)}")


class CarrotTuningLayoutTests(unittest.TestCase):
  """The carrot tuning page is a root list of groups, not a tab strip.

  It used to be the only panel in the catalogue using the `tabs` widget. It now
  mirrors the native settings page: a root list of subpanel rows, one per group,
  separated by `separator` (the webui equivalent of LineSeparatorSP).
  """

  ROOT = "carrot"

  def test_root_uses_subpanel_rows_not_tabs(self):
    widgets = get_panel(self.ROOT)["widgets"]
    types = [w.get("type") for w in widgets]
    self.assertNotIn("tabs", types, "the tab strip is gone; use subpanel rows")
    self.assertNotIn("tab", types, "tab panes are gone; use subpanel rows")

  def test_every_subpanel_row_targets_a_real_panel(self):
    widgets = get_panel(self.ROOT)["widgets"]
    rows = [w for w in widgets if w.get("type") == "subpanel"]
    # 8 groups: "Path Rendering" lost every item when the params with no reader
    # were hidden, so the group and its row were removed rather than left empty.
    self.assertEqual(len(rows), 8, f"expected 8 group rows, got {len(rows)}")
    for w in rows:
      target = w["target"]
      self.assertIn(target, SUBPANELS, f"{target} is not a registered subpanel")
      self.assertTrue(get_panel(target), f"{target} resolves to nothing")
      self.assertEqual(get_panel(target).get("parent"), self.ROOT,
                       f"{target} must declare parent={self.ROOT} so the back button renders")

  def test_group_rows_are_separated_and_described(self):
    widgets = get_panel(self.ROOT)["widgets"]
    row_indices = [i for i, w in enumerate(widgets) if w.get("type") == "subpanel"]
    self.assertEqual(len(row_indices), 8, "expected 8 group rows")
    self.assertNotEqual(widgets[0].get("type"), "separator", "the list must not open with a separator")
    for i in range(len(row_indices) - 1):
      between = widgets[row_indices[i] + 1:row_indices[i + 1]]
      self.assertEqual(sum(1 for w in between if w.get("type") == "separator"), 1,
                       "exactly one separator between each pair of group rows")
    for idx in row_indices:
      w = widgets[idx]
      self.assertTrue(w.get("desc"), f"{w['target']} has no description")
      self.assertTrue(w.get("label"), f"{w['target']} has no label")

  def test_no_group_panel_is_empty(self):
    widgets = get_panel(self.ROOT)["widgets"]
    for w in (x for x in widgets if x.get("type") == "subpanel"):
      sub = get_panel(w["target"])["widgets"]
      self.assertTrue(sub, f"{w['target']} has no widgets")
class CarrotHiddenParamsAreReallyDeadTests(unittest.TestCase):
  """A param hidden from the UI must not be read anywhere in the tree.

  The first version of the dead-param scan only walked `openpilot/**`, so readers
  living inside the submodules were invisible and four working params got hidden -
  three read by opendbc's BYD port (opendbc_repo/opendbc/car/byd/carcontroller.py and
  radar_interface.py) and one by carrot_serv. This test scans the submodule trees too.
  """

  # Every source tree that ships and can read a param. `openpilot` is included so the
  # test also catches a hidden param that gained a reader since it was hidden.
  ROOTS = ("openpilot", "opendbc_repo", "panda", "msgq_repo", "system")
  EXTS = (".py", ".cc", ".h", ".cpp", ".hpp", ".pyx")
  # files that legitimately name every param without reading it
  SURFACES = ("carrot_tuning_items.py", "panel_catalog.py", "carrot/config.py",
              "carrot_tuning_api.py", "nav_params.json", "params_keys.h",
              "carrot_tuning.py")

  def _repo_root(self):
    from pathlib import Path
    # .../webui/server/bridge/tests/ -> repo root
    return Path(__file__).resolve().parents[4]

  def test_no_hidden_carrot_param_has_a_reader(self):
    root = self._repo_root()
    names = set(CARROT_TUNING_UNAVAILABLE)
    self.assertTrue(names, "CARROT_TUNING_UNAVAILABLE is empty; the scan lost its input")

    offenders: dict[str, list[str]] = {}
    for tree in self.ROOTS:
      for ext in self.EXTS:
        for path in root.joinpath(tree).rglob(f"*{ext}"):
          q = str(path).replace("\\", "/")
          if "__pycache__" in q or "/tests/" in q or path.name.startswith("test_"):
            continue
          if any(s in q for s in self.SURFACES):
            continue
          try:
            text = path.read_text(encoding="utf-8")
          except (UnicodeDecodeError, OSError):
            continue
          for name in names:
            pattern = r"get\w*\s*\(\s*[\"']" + re.escape(name) + r"[\"']"
            if re.search(pattern, text):
              rel = q.split("/openpilot/", 1)[-1]
              offenders.setdefault(name, []).append(rel)

    self.assertFalse(
      offenders,
      "these params are hidden from the UI but still read by shipped code - "
      f"unhide them or confirm the reader is gone: {offenders}",
    )


if __name__ == "__main__":
  unittest.main()
