#!/usr/bin/env python3
"""Tests for the canonical Crazyflie vehicle configuration."""

import importlib.util
import math
import sys
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SCRIPT_DIR.parent / "launch" / "crazyflies.yaml"
CONTROLLER_CONFIG_PATH = SCRIPT_DIR.parent / "config" / "ctbr_controller.yaml"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def _cf_arm_module():
    spec = importlib.util.spec_from_file_location("cf_arm_under_test", SCRIPT_DIR / "cf_arm.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_canonical_vehicle_entry_has_explicit_uri_and_ctbr_selection():
    module = _cf_arm_module()
    config = yaml.safe_load(CONFIG_PATH.read_text())
    entries = config["crazyflies"]

    selected = module.select_vehicle_entry(entries, cf_id=2)

    assert selected["id"] == 2
    assert selected["uri"] == "radio://0/80/2M/E7E7E7E702"
    assert selected["ctbr_enabled"] is True
    assert module.vehicle_uri(selected) == selected["uri"]

    selected_entries = module.select_vehicle_entries(entries)
    assert [entry["id"] for entry in selected_entries] == [2, 4, 5]
    validated = module.validate_vehicle_entries(
        entries, require_shared_radio=True, require_phase=True
    )
    assert [entry["id"] for entry in validated] == [2, 4, 5]
    assert [entry["uri"] for entry in validated] == [
        "radio://0/80/2M/E7E7E7E702",
        "radio://0/80/2M/E7E7E7E704",
        "radio://0/80/2M/E7E7E7E705",
    ]
    assert [entry["orbit_phase_rad"] for entry in validated] == [
        0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0,
    ]


def test_vehicle_uri_keeps_legacy_fallback_for_old_entries():
    module = _cf_arm_module()

    assert module.vehicle_uri({"id": 3, "channel": 80}) == (
        "radio://0/80/2M/E7E7E7E703"
    )


def test_vehicle_selection_by_id_remains_available_in_multi_configuration():
    module = _cf_arm_module()
    config = yaml.safe_load(CONFIG_PATH.read_text())

    selected = module.select_vehicle_entry(config["crazyflies"], cf_id=4)

    assert selected["id"] == 4


def test_multi_vehicle_validation_rejects_mismatched_channel():
    module = _cf_arm_module()
    entries = [
        {"id": 2, "channel": 80, "uri": "radio://0/80/2M/E7E7E7E702",
         "ctbr_enabled": True, "orbit_phase_rad": 0.0},
        {"id": 4, "channel": 81, "uri": "radio://0/81/2M/E7E7E7E704",
         "ctbr_enabled": True, "orbit_phase_rad": 3.14},
    ]
    try:
        module.validate_vehicle_entries(entries, require_shared_radio=True,
                                       require_phase=True)
    except ValueError as error:
        assert "同一 Crazyradio/channel" in str(error)
    else:
        raise AssertionError("mismatched channels must be rejected")


def test_enabled_vehicles_have_dedicated_controller_parameter_blocks():
    """An enabled CF must never inherit another CF's tuning by accident."""
    vehicles = yaml.safe_load(CONFIG_PATH.read_text())["crazyflies"]
    controller_config = yaml.safe_load(CONTROLLER_CONFIG_PATH.read_text())

    assert "ctbr_trajectory" in controller_config
    enabled_ids = [entry["id"] for entry in vehicles if entry["ctbr_enabled"]]
    for vehicle_id in enabled_ids:
        block = controller_config["ctbr_controller_cf%d" % vehicle_id]
        assert block["mass_kg"] > 0.0
        assert block["max_total_thrust_newton"] >= block["max_command_thrust_newton"]
        assert len(block["position_gain"]) == 3

    assert "mass_kg" not in controller_config["ctbr_controller"]
    assert "trajectory_mode" not in controller_config["ctbr_controller"]


def test_every_vehicle_declares_formation_gains_explicitly():
    """Formation gains must not fall back to a shared global default.

    The global block intentionally no longer defines these keys.  If a vehicle
    block omitted one, the resolver would silently fall through to the global
    block and then to a hardcoded nominal value, so a newly added aircraft
    could fly with a completely different integral gain without any error.
    """
    vehicles = yaml.safe_load(CONFIG_PATH.read_text())["crazyflies"]
    controller_config = yaml.safe_load(CONTROLLER_CONFIG_PATH.read_text())
    global_block = controller_config["ctbr_controller"]

    gains = [
        "formation_bl",
        "formation_kf",
        "formation_kvf",
        "formation_kbl",
        "formation_kvl",
        "formation_kil",
        "formation_integral_c1",
        "formation_integral_limit_m",
    ]

    for gain in gains:
        assert gain not in global_block, (
            "%s 不应出现在全局块：逐机声明才能避免静默继承" % gain
        )

    for entry in vehicles:
        if not entry["ctbr_enabled"]:
            continue
        block = controller_config["ctbr_controller_cf%d" % entry["id"]]
        for gain in gains:
            assert gain in block, (
                "CF%d 缺少 %s；必须在自己的参数块里显式声明"
                % (entry["id"], gain)
            )


def test_adjacency_matrix_index_order_matches_the_launch_order():
    """``formation_adjacency`` is indexed by launch order, not by CF ID.

    ``ctbr_controller.py`` reads ``formation_adjacency[i, j]`` where ``i`` and
    ``j`` are positions in ``self.vehicles``, and that list follows the order
    the ``ctbr_enabled`` entries appear in ``crazyflies.yaml``.  Reordering the
    fleet therefore silently re-targets every edge of the communication graph,
    and the matrix alone carries no clue about which row means which
    aircraft.  This test pins the order that the comment above the matrix
    documents, so a reorder fails here and forces the matrix, its comment and
    this expectation to be updated together.
    """
    vehicles = yaml.safe_load(CONFIG_PATH.read_text())["crazyflies"]
    controller_config = yaml.safe_load(CONTROLLER_CONFIG_PATH.read_text())
    adjacency = controller_config["ctbr_controller"]["formation_adjacency"]

    enabled_order = [entry["id"] for entry in vehicles if entry["ctbr_enabled"]]

    # Keep in sync with the comment above ``formation_adjacency`` in the YAML.
    assert enabled_order == [2, 4, 5], (
        "启用顺序已变为 %s；必须同步更新 formation_adjacency 的行列顺序、"
        "YAML 中说明该顺序的注释，以及本断言" % enabled_order
    )

    size = len(enabled_order)
    assert len(adjacency) == size, "邻接矩阵行数必须等于启用飞机数量"
    for row in adjacency:
        assert len(row) == size, "邻接矩阵必须是方阵"

    for index in range(size):
        assert adjacency[index][index] == 0.0, (
            "邻接矩阵对角线（自己与自己）必须为 0；代码也会强制置 0"
        )
        for weight in adjacency[index]:
            assert weight >= 0.0, "邻接权重不能为负"
