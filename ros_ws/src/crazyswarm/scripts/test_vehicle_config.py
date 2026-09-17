#!/usr/bin/env python3
"""Tests for the canonical Crazyflie vehicle configuration."""

import importlib.util
import sys
from pathlib import Path

import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_PATH = SCRIPT_DIR.parent / "launch" / "crazyflies.yaml"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def _cf_arm_module():
    spec = importlib.util.spec_from_file_location("cf_arm_under_test", SCRIPT_DIR / "cf_arm.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _enabled_entries():
    """Return the ``ctbr_enabled`` entries of the shipped configuration.

    The active aircraft ID is a deployment choice: README documents swapping
    it by editing only ``crazyflies.yaml``.  Tests therefore read the ID and
    URI from that file instead of hardcoding one, so switching aircraft does
    not silently break the suite.
    """
    config = yaml.safe_load(CONFIG_PATH.read_text())
    entries = config["crazyflies"]
    return entries, [e for e in entries if bool(e.get("ctbr_enabled", False))]


def test_canonical_vehicle_entry_has_explicit_uri_and_ctbr_selection():
    module = _cf_arm_module()
    entries, enabled = _enabled_entries()

    assert len(enabled) == 1, "单机分支必须且只能启用一架飞机"
    expected_id = int(enabled[0]["id"])

    selected = module.select_vehicle_entry(entries, cf_id=expected_id)

    assert selected["id"] == expected_id
    assert selected["uri"] == enabled[0]["uri"]
    assert selected["ctbr_enabled"] is True
    assert module.vehicle_uri(selected) == selected["uri"]


def test_vehicle_uri_keeps_legacy_fallback_for_old_entries():
    module = _cf_arm_module()

    assert module.vehicle_uri({"id": 3, "channel": 80}) == (
        "radio://0/80/2M/E7E7E7E703"
    )


def test_vehicle_selection_requires_one_ctbr_entry_when_id_is_omitted():
    module = _cf_arm_module()
    entries, enabled = _enabled_entries()

    selected = module.select_vehicle_entry(entries)

    assert selected["id"] == int(enabled[0]["id"])
