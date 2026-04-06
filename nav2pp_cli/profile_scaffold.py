from __future__ import annotations

import json
from pathlib import Path

from .validator import (
    DEFAULT_EXPECTED_TOPIC_TYPES,
    DEFAULT_RESOURCE_HINTS,
    ResourceEndpoint,
    _normalize_name,
    _resolve_endpoint,
    load_topic_snapshot,
)


DBW_HINTS = [
    "/vehicle/dbw/command",
    "/dbw/command",
    "/vehicle/ackermann_cmd",
    "/ackermann_cmd",
    "/vehicle/cmd_ackermann",
    "/cmd_ackermann",
]

DBW_TYPES = [
    "ackermann_msgs/msg/AckermannDriveStamped",
    "geometry_msgs/msg/Twist",
]

OPTIONAL_ROLES = ["goal_pose", "imu", "gps_fix"]
DISCOVERY_ROLES = ["map", "odom", "tf", "tf_static", "cmd_vel", *OPTIONAL_ROLES, "scan", "pointcloud"]


def scaffold_profile(
    name: str,
    repo_root: Path,
    topics: list[str] | None = None,
    topic_file: Path | None = None,
    output_path: Path | None = None,
    require_nvidia: bool = True,
    force: bool = False,
) -> tuple[Path, list[str]]:
    repo_root = repo_root.resolve()
    profile_name = name.strip().lower()
    if not profile_name:
        raise ValueError("Profile name must not be empty.")

    destination = _resolve_output_path(profile_name, repo_root, output_path)
    if destination.exists() and not force:
        raise ValueError(f"Refusing to overwrite existing profile without --force: {destination}")

    endpoints = load_topic_snapshot(topics=topics, topic_file=topic_file)
    payload, notes = _build_profile_payload(profile_name, endpoints, require_nvidia=require_nvidia)

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    return destination, notes


def _resolve_output_path(profile_name: str, repo_root: Path, output_path: Path | None) -> Path:
    if output_path is None:
        return repo_root / "profiles" / f"{profile_name}.json"
    return output_path if output_path.is_absolute() else repo_root / output_path


def _build_profile_payload(
    profile_name: str,
    endpoints: dict[str, ResourceEndpoint],
    require_nvidia: bool,
) -> tuple[dict[str, object], list[str]]:
    notes: list[str] = []
    resource_hints: dict[str, list[str]] = {}
    expected_topic_types: dict[str, list[str]] = {}

    resolved_by_role = {
        role: _resolve_endpoint(endpoints, DEFAULT_RESOURCE_HINTS.get(role, []))
        for role in DISCOVERY_ROLES
    }
    dbw_endpoint = _resolve_dbw_endpoint(endpoints)

    for role, endpoint in resolved_by_role.items():
        if endpoint is None:
            continue
        hints = _merge_hints(endpoint.name, DEFAULT_RESOURCE_HINTS.get(role, []))
        if hints and hints != [_normalize_name(item) for item in DEFAULT_RESOURCE_HINTS.get(role, [])]:
            resource_hints[role] = hints

    if dbw_endpoint is not None:
        resource_hints["dbw_cmd"] = _merge_hints(dbw_endpoint.name, DBW_HINTS)
        if dbw_endpoint.msg_type:
            expected_topic_types["dbw_cmd"] = [dbw_endpoint.msg_type]
    else:
        resource_hints["dbw_cmd"] = [_normalize_name(item) for item in DBW_HINTS]
        expected_topic_types["dbw_cmd"] = DBW_TYPES
        notes.append("No DBW command topic was auto-detected. Review resource_hints.dbw_cmd.")

    required_sensor_roles = [role for role in ["scan", "pointcloud"] if resolved_by_role[role] is not None]
    if not required_sensor_roles:
        required_sensor_roles = ["scan", "pointcloud"]
        notes.append("No sensor input was auto-detected. Review required_sensor_roles and resource_hints.")

    if resolved_by_role["odom"] is None:
        notes.append("No odometry topic was auto-detected. Review resource_hints.odom.")
    if resolved_by_role["map"] is None:
        notes.append("No map topic was auto-detected. Review resource_hints.map.")

    payload: dict[str, object] = {
        "name": profile_name,
        "extends": "vehicle",
        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel", "dbw_cmd"],
        "recommended_roles": OPTIONAL_ROLES,
        "required_sensor_roles": required_sensor_roles,
        "resource_hints": resource_hints,
        "expected_topic_types": expected_topic_types,
    }

    if require_nvidia:
        payload["host_checks"] = [
            {
                "name": "nvidia_gpu",
                "command": ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                "nonempty_stdout": True,
                "required": True,
            }
        ]

    if notes:
        payload["notes"] = notes
    return payload, notes


def _resolve_dbw_endpoint(endpoints: dict[str, ResourceEndpoint]) -> ResourceEndpoint | None:
    exact = _resolve_endpoint(endpoints, DBW_HINTS)
    if exact is not None:
        return exact

    candidates: list[ResourceEndpoint] = []
    for endpoint in endpoints.values():
        normalized = endpoint.name.lower()
        if "dbw" in normalized or "ackermann" in normalized:
            candidates.append(endpoint)
            continue
        if normalized.endswith("/command") and ("vehicle" in normalized or "drive" in normalized):
            candidates.append(endpoint)
    if not candidates:
        return None

    typed = [
        endpoint
        for endpoint in candidates
        if endpoint.msg_type in DBW_TYPES
    ]
    return typed[0] if typed else candidates[0]


def _merge_hints(primary: str, fallbacks: list[str]) -> list[str]:
    merged: list[str] = []
    for item in [primary, *fallbacks]:
        normalized = _normalize_name(item)
        if normalized not in merged:
            merged.append(normalized)
    return merged
