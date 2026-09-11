#!/usr/bin/env python3
"""Shared loader and selector for the canonical Crazyflie vehicle YAML."""

import os

import yaml


def select_vehicle_entry(entries, cf_id=None):
    """Select one vehicle entry from a ``crazyflies`` YAML list.

    When ``cf_id`` is omitted, exactly one entry must opt into the CTBR
    controller with ``ctbr_enabled: true``.  This prevents an accidental
    multi-aircraft launch from silently selecting the first entry.
    """
    if not isinstance(entries, list) or not entries:
        raise ValueError("crazyflies.yaml 必须包含非空 crazyflies 列表")
    if cf_id is not None:
        matches = [entry for entry in entries if int(entry.get("id", -1)) == int(cf_id)]
        if len(matches) != 1:
            raise ValueError("crazyflies.yaml 中找不到唯一的 cf%d" % int(cf_id))
        return matches[0]
    matches = [entry for entry in entries if bool(entry.get("ctbr_enabled", False))]
    if len(matches) != 1:
        raise ValueError(
            "未指定 cf_id 时，crazyflies.yaml 必须且只能有一个 ctbr_enabled=true 的飞机"
        )
    return matches[0]


def load_vehicle_entry(config_path, cf_id=None):
    """Load and validate one vehicle entry from a YAML parameter file."""
    with open(os.path.expanduser(config_path), "r") as config_file:
        config = yaml.safe_load(config_file) or {}
    entry = select_vehicle_entry(config.get("crazyflies"), cf_id=cf_id)
    if not str(entry.get("uri", "")).strip():
        raise ValueError("选中的飞机配置缺少非空 uri")
    return entry


def vehicle_uri(entry):
    """Return the configured URI, or the legacy URI derived from id/channel."""
    configured = str(entry.get("uri", "")).strip()
    if configured:
        return configured
    try:
        channel = int(entry["channel"])
        cf_id = int(entry["id"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("飞机条目缺少有效的 uri 或 id/channel") from exc
    if not 0 <= cf_id <= 0xFF:
        raise ValueError("飞机 id 必须在 0..255 范围内")
    return "radio://0/{}/2M/E7E7E7E7{:02X}".format(channel, cf_id)
