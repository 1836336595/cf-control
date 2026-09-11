#!/usr/bin/env python3
"""Shared loader and selector for the canonical Crazyflie vehicle YAML."""

import os
import re

import yaml


def _normalise_entries(entries):
    """Validate the shared vehicle list and return shallow copies."""
    if not isinstance(entries, list) or not entries:
        raise ValueError("crazyflies.yaml 必须包含非空 crazyflies 列表")
    result = []
    ids = set()
    uris = set()
    for index, raw_entry in enumerate(entries):
        if not isinstance(raw_entry, dict):
            raise ValueError("crazyflies[%d] 必须是映射" % index)
        try:
            cf_id = int(raw_entry["id"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("crazyflies[%d] 缺少有效 id" % index) from exc
        if not 0 <= cf_id <= 0xFF:
            raise ValueError("飞机 id 必须在 0..255 范围内")
        if cf_id in ids:
            raise ValueError("飞机 id 重复：cf%d" % cf_id)
        ids.add(cf_id)
        entry = dict(raw_entry)
        entry["id"] = cf_id
        configured_uri = str(entry.get("uri", "")).strip()
        if configured_uri:
            if configured_uri in uris:
                raise ValueError("飞机 URI 重复：%s" % configured_uri)
            uris.add(configured_uri)
            entry["uri"] = configured_uri
        if "orbit_phase_rad" in entry:
            try:
                phase = float(entry["orbit_phase_rad"])
            except (TypeError, ValueError) as exc:
                raise ValueError("cf%d 的 orbit_phase_rad 无效" % cf_id) from exc
            if phase != phase or phase in (float("inf"), float("-inf")):
                raise ValueError("cf%d 的 orbit_phase_rad 必须有限" % cf_id)
            entry["orbit_phase_rad"] = phase
        result.append(entry)
    return result


def select_vehicle_entries(entries, cf_id=None, ctbr_only=True):
    """Select one or all configured vehicles for a controller operation.

    With no explicit ``cf_id``, all ``ctbr_enabled`` entries are returned in
    YAML order.  This is the multi-vehicle counterpart of
    :func:`select_vehicle_entry` and intentionally keeps the old selector
    behavior for single-vehicle callers.
    """
    normalised = _normalise_entries(entries)
    if cf_id is not None:
        matches = [entry for entry in normalised if entry["id"] == int(cf_id)]
        if len(matches) != 1:
            raise ValueError("crazyflies.yaml 中找不到唯一的 cf%d" % int(cf_id))
        return matches
    if ctbr_only:
        matches = [entry for entry in normalised if bool(entry.get("ctbr_enabled", False))]
    else:
        matches = normalised
    if not matches:
        raise ValueError("没有启用 ctbr_enabled=true 的飞机")
    return matches


def validate_vehicle_entries(entries, require_ctbr=True, min_ctbr=1,
                             require_shared_radio=False, require_phase=False):
    """Validate a selected fleet before a synchronized flight.

    The Crazyswarm server still owns the actual radio connections.  This
    helper only catches configuration mistakes early in the host controller:
    missing channels/URIs, duplicate identities, and (for a one-radio task)
    entries that do not share the same ``radio://<radio>/<channel>`` prefix.
    """
    normalised = _normalise_entries(entries)
    selected = [
        entry for entry in normalised
        if (not require_ctbr or bool(entry.get("ctbr_enabled", False)))
    ]
    if len(selected) < int(min_ctbr):
        raise ValueError(
            "至少需要 %d 架 ctbr_enabled=true 的飞机，当前为 %d" %
            (int(min_ctbr), len(selected))
        )

    channels = set()
    radio_channels = set()
    for entry in selected:
        cf_id = int(entry["id"])
        try:
            channel = int(entry["channel"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("cf%d 缺少有效 channel" % cf_id) from exc
        if not 0 <= channel <= 125:
            raise ValueError("cf%d 的 channel 必须在 0..125 范围内" % cf_id)
        channels.add(channel)

        uri = vehicle_uri(entry).strip()
        if not uri:
            raise ValueError("cf%d 缺少非空 uri" % cf_id)
        match = re.match(r"^radio://([^/]+)/([^/]+)/", uri)
        if require_shared_radio:
            if match is None:
                raise ValueError("cf%d 的 URI 不是可验证的 radio:// 地址：%s" %
                                 (cf_id, uri))
            if int(match.group(2)) != channel:
                raise ValueError(
                    "cf%d 的 URI channel=%s 与配置 channel=%d 不一致" %
                    (cf_id, match.group(2), channel)
                )
            radio_channels.add((match.group(1), match.group(2)))
        if require_phase and "orbit_phase_rad" not in entry:
            raise ValueError("多机任务中 cf%d 必须配置 orbit_phase_rad" % cf_id)

    if require_shared_radio and (len(channels) > 1 or len(radio_channels) > 1):
        raise ValueError(
            "多机 CTBR 必须使用同一 Crazyradio/channel；当前 channel=%s，URI=%s" %
            (sorted(channels), sorted(radio_channels))
        )
    return selected


def select_vehicle_entry(entries, cf_id=None):
    """Select one vehicle entry from a ``crazyflies`` YAML list.

    When ``cf_id`` is omitted, exactly one entry must opt into the CTBR
    controller with ``ctbr_enabled: true``.  This prevents an accidental
    multi-aircraft launch from silently selecting the first entry.
    """
    matches = select_vehicle_entries(entries, cf_id=cf_id, ctbr_only=True)
    if cf_id is not None:
        return matches[0]
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


def load_vehicle_entries(config_path, cf_id=None, ctbr_only=True):
    """Load all selected vehicle entries from a YAML parameter file."""
    with open(os.path.expanduser(config_path), "r") as config_file:
        config = yaml.safe_load(config_file) or {}
    entries = select_vehicle_entries(
        config.get("crazyflies"), cf_id=cf_id, ctbr_only=ctbr_only
    )
    for entry in entries:
        if not vehicle_uri(entry).strip():
            raise ValueError("cf%d 缺少非空 uri" % int(entry["id"]))
    return entries


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
