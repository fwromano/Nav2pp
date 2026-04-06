from __future__ import annotations

import json
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .topics import TOPIC_HINTS


RESOURCE_HINTS = {
    **TOPIC_HINTS,
    "imu": ["/imu", "imu", "/imu/data", "imu/data"],
    "gps_fix": ["/fix", "fix", "/gps/fix", "gps/fix", "/gps/filtered"],
}

EXPECTED_TOPIC_TYPES = {
    "map": {"nav_msgs/msg/OccupancyGrid"},
    "odom": {"nav_msgs/msg/Odometry"},
    "scan": {"sensor_msgs/msg/LaserScan"},
    "pointcloud": {"sensor_msgs/msg/PointCloud2"},
    "cmd_vel": {"geometry_msgs/msg/Twist"},
    "tf": {"tf2_msgs/msg/TFMessage"},
    "tf_static": {"tf2_msgs/msg/TFMessage"},
    "goal_pose": {"geometry_msgs/msg/PoseStamped"},
    "imu": {"sensor_msgs/msg/Imu"},
    "gps_fix": {"sensor_msgs/msg/NavSatFix"},
}

PROFILE_SPECS = {
    "vehicle": {
        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel"],
        "recommended_roles": ["goal_pose", "imu", "gps_fix"],
        "required_sensor_roles": ["scan", "pointcloud"],
        "required_actions": [],
    },
    "nav2": {
        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel"],
        "recommended_roles": ["goal_pose", "imu"],
        "required_sensor_roles": ["scan", "pointcloud"],
        "required_actions": [
            ("/navigate_to_pose", {"nav2_msgs/action/NavigateToPose"}),
        ],
    },
}


@dataclass
class ResourceEndpoint:
    name: str
    msg_type: str | None


@dataclass
class ValidationCheck:
    name: str
    status: str
    detail: str
    required: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ValidationReport:
    profile: str
    checks: list[ValidationCheck]
    warnings: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not any(check.required and check.status == "fail" for check in self.checks)

    def to_dict(self) -> dict[str, Any]:
        return {
            "profile": self.profile,
            "passed": self.passed,
            "checks": [check.to_dict() for check in self.checks],
            "warnings": list(self.warnings),
        }


def validate_profile(
    profile: str,
    topics: list[str] | None = None,
    topic_file: Path | None = None,
) -> ValidationReport:
    profile_name = profile.lower()
    if profile_name not in PROFILE_SPECS:
        raise ValueError(f"Unknown validation profile: {profile}")

    endpoints = load_topic_snapshot(topics=topics, topic_file=topic_file)
    checks: list[ValidationCheck] = []
    warnings: list[str] = []
    live_probe = topics is None and topic_file is None

    spec = PROFILE_SPECS[profile_name]
    for role in spec["required_roles"]:
        checks.append(_topic_role_check(role, endpoints, required=True))
    for role in spec["recommended_roles"]:
        checks.append(_topic_role_check(role, endpoints, required=False))
    checks.append(_sensor_check(endpoints, roles=spec["required_sensor_roles"], required=True))

    if live_probe:
        _add_frame_checks(checks, endpoints)
        _add_tf_checks(checks)
        if spec["required_actions"]:
            actions = _load_action_snapshot()
            for action_name, expected_types in spec["required_actions"]:
                checks.append(_action_check(actions, action_name, expected_types))
    else:
        warnings.append(
            "Live frame, TF, and action probes were skipped because a topic snapshot was supplied."
        )

    return ValidationReport(profile=profile_name, checks=checks, warnings=warnings)


def render_validation_report(report: ValidationReport, as_json: bool) -> str:
    if as_json:
        return json.dumps(report.to_dict(), indent=2, sort_keys=True)

    required_total = sum(1 for check in report.checks if check.required)
    required_passed = sum(1 for check in report.checks if check.required and check.status == "pass")
    lines = [
        f"Profile: {report.profile}",
        f"Result: {'pass' if report.passed else 'fail'} ({required_passed}/{required_total} required checks passed)",
        "Checks:",
    ]
    for check in report.checks:
        flag = check.status.upper().ljust(4)
        requirement = "required" if check.required else "optional"
        lines.append(f"  - {flag} {check.name} [{requirement}] {check.detail}")
    if report.warnings:
        lines.append("Warnings:")
        for warning in report.warnings:
            lines.append(f"  - {warning}")
    return "\n".join(lines)


def load_topic_snapshot(
    topics: list[str] | None = None,
    topic_file: Path | None = None,
) -> dict[str, ResourceEndpoint]:
    if topics is not None:
        return _parse_topic_entries(topics)
    if topic_file is not None:
        return _load_topic_file(topic_file)
    return _run_ros2_topic_list_t()


def _load_topic_file(topic_file: Path) -> dict[str, ResourceEndpoint]:
    text = topic_file.read_text()
    stripped = text.strip()
    if not stripped:
        return {}
    if stripped.startswith("{") or stripped.startswith("["):
        payload = json.loads(stripped)
        return _parse_json_topic_payload(payload)
    return _parse_topic_entries(text.splitlines())


def _parse_json_topic_payload(payload: Any) -> dict[str, ResourceEndpoint]:
    endpoints: dict[str, ResourceEndpoint] = {}
    if isinstance(payload, dict):
        topics = payload.get("topics", payload)
        if isinstance(topics, dict):
            for name, msg_type in topics.items():
                normalized = _normalize_name(str(name))
                endpoints[normalized] = ResourceEndpoint(name=normalized, msg_type=_normalize_type(msg_type))
            return endpoints
        payload = topics
    if isinstance(payload, list):
        for item in payload:
            if isinstance(item, str):
                endpoint = _parse_topic_entry(item)
            elif isinstance(item, dict):
                endpoint = ResourceEndpoint(
                    name=_normalize_name(str(item.get("name", ""))),
                    msg_type=_normalize_type(item.get("type")),
                )
            else:
                continue
            if endpoint.name:
                endpoints[endpoint.name] = endpoint
    return endpoints


def _parse_topic_entries(entries: list[str]) -> dict[str, ResourceEndpoint]:
    endpoints: dict[str, ResourceEndpoint] = {}
    for entry in entries:
        endpoint = _parse_topic_entry(entry)
        if endpoint.name:
            endpoints[endpoint.name] = endpoint
    return endpoints


def _parse_topic_entry(entry: str) -> ResourceEndpoint:
    stripped = entry.strip()
    if not stripped:
        return ResourceEndpoint(name="", msg_type=None)
    if "[" in stripped and stripped.endswith("]"):
        name, raw_type = stripped.split("[", 1)
        return ResourceEndpoint(name=_normalize_name(name), msg_type=_normalize_type(raw_type[:-1].strip()))
    if ":" in stripped:
        name, raw_type = stripped.split(":", 1)
        return ResourceEndpoint(name=_normalize_name(name), msg_type=_normalize_type(raw_type))
    return ResourceEndpoint(name=_normalize_name(stripped), msg_type=None)


def _normalize_name(name: str) -> str:
    cleaned = name.strip()
    if not cleaned:
        return ""
    if not cleaned.startswith("/"):
        cleaned = f"/{cleaned}"
    return cleaned


def _normalize_type(raw_type: Any) -> str | None:
    if raw_type is None:
        return None
    cleaned = str(raw_type).strip()
    return cleaned or None


def _run_ros2_topic_list_t() -> dict[str, ResourceEndpoint]:
    try:
        completed = subprocess.run(
            ["ros2", "topic", "list", "-t"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("ros2 is not installed or not on PATH") from exc
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "ros2 topic list -t failed"
        raise RuntimeError(stderr)
    return _parse_topic_entries(completed.stdout.splitlines())


def _load_action_snapshot() -> dict[str, ResourceEndpoint]:
    try:
        completed = subprocess.run(
            ["ros2", "action", "list", "-t"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return {}
    if completed.returncode != 0:
        return {}
    return _parse_topic_entries(completed.stdout.splitlines())


def _topic_role_check(
    role: str,
    endpoints: dict[str, ResourceEndpoint],
    required: bool,
) -> ValidationCheck:
    endpoint = _resolve_endpoint(endpoints, RESOURCE_HINTS.get(role, []))
    if endpoint is None:
        return ValidationCheck(
            name=role,
            status="fail" if required else "warn",
            detail="missing from the ROS graph",
            required=required,
        )

    expected_types = EXPECTED_TOPIC_TYPES.get(role, set())
    if expected_types and endpoint.msg_type and endpoint.msg_type not in expected_types:
        return ValidationCheck(
            name=role,
            status="fail" if required else "warn",
            detail=f"{endpoint.name} has type {endpoint.msg_type}, expected one of {sorted(expected_types)}",
            required=required,
        )

    if expected_types and endpoint.msg_type is None:
        return ValidationCheck(
            name=role,
            status="warn",
            detail=f"{endpoint.name} was detected, but its message type is unknown",
            required=required,
        )

    return ValidationCheck(
        name=role,
        status="pass",
        detail=f"{endpoint.name}" + (f" [{endpoint.msg_type}]" if endpoint.msg_type else ""),
        required=required,
    )


def _sensor_check(
    endpoints: dict[str, ResourceEndpoint],
    roles: list[str],
    required: bool,
) -> ValidationCheck:
    resolved = [
        (role, _resolve_endpoint(endpoints, RESOURCE_HINTS.get(role, [])))
        for role in roles
    ]
    if all(endpoint is None for _, endpoint in resolved):
        return ValidationCheck(
            name="sensor_input",
            status="fail" if required else "warn",
            detail="missing all required sensor inputs",
            required=required,
        )

    for role, endpoint in resolved:
        if endpoint is None:
            continue
        expected_types = EXPECTED_TOPIC_TYPES[role]
        if endpoint.msg_type is None or endpoint.msg_type in expected_types:
            return ValidationCheck(
                name="sensor_input",
                status="pass",
                detail=f"using {endpoint.name}" + (f" [{endpoint.msg_type}]" if endpoint.msg_type else ""),
                required=required,
            )

    return ValidationCheck(
        name="sensor_input",
        status="fail" if required else "warn",
        detail=(
            "detected sensor topics but with unexpected types: "
            + ", ".join(
                sorted(
                    {
                        endpoint.msg_type or "unknown"
                        for _, endpoint in resolved
                        if endpoint is not None
                    }
                )
            )
        ),
        required=required,
    )


def _resolve_endpoint(endpoints: dict[str, ResourceEndpoint], hints: list[str]) -> ResourceEndpoint | None:
    names = list(endpoints)
    for hint in hints:
        candidate = _normalize_name(hint)
        if candidate in endpoints:
            return endpoints[candidate]
    for hint in hints:
        normalized = hint.lstrip("/")
        for name in names:
            if name.lstrip("/").endswith(normalized):
                return endpoints[name]
    return None


def _add_frame_checks(checks: list[ValidationCheck], endpoints: dict[str, ResourceEndpoint]) -> None:
    map_endpoint = _resolve_endpoint(endpoints, RESOURCE_HINTS["map"])
    odom_endpoint = _resolve_endpoint(endpoints, RESOURCE_HINTS["odom"])
    scan_endpoint = _resolve_endpoint(endpoints, RESOURCE_HINTS["scan"])
    pointcloud_endpoint = _resolve_endpoint(endpoints, RESOURCE_HINTS["pointcloud"])

    if map_endpoint is not None:
        frame = _read_topic_field_once(map_endpoint.name, "header.frame_id")
        checks.append(
            ValidationCheck(
                name="map_frame",
                status="pass" if frame == "map" else "fail",
                detail=f"{map_endpoint.name} header.frame_id={frame!r}" if frame else f"could not read header.frame_id from {map_endpoint.name}",
                required=True,
            )
        )
    if odom_endpoint is not None:
        odom_frame = _read_topic_field_once(odom_endpoint.name, "header.frame_id")
        checks.append(
            ValidationCheck(
                name="odom_frame",
                status="pass" if odom_frame == "odom" else "fail",
                detail=f"{odom_endpoint.name} header.frame_id={odom_frame!r}" if odom_frame else f"could not read header.frame_id from {odom_endpoint.name}",
                required=True,
            )
        )
        child_frame = _read_topic_field_once(odom_endpoint.name, "child_frame_id")
        checks.append(
            ValidationCheck(
                name="base_frame",
                status="pass" if child_frame in {"base_link", "base_footprint"} else "fail",
                detail=f"{odom_endpoint.name} child_frame_id={child_frame!r}" if child_frame else f"could not read child_frame_id from {odom_endpoint.name}",
                required=True,
            )
        )
    sensor_endpoint = scan_endpoint or pointcloud_endpoint
    if sensor_endpoint is not None:
        frame = _read_topic_field_once(sensor_endpoint.name, "header.frame_id")
        checks.append(
            ValidationCheck(
                name="sensor_frame",
                status="pass" if frame else "warn",
                detail=f"{sensor_endpoint.name} header.frame_id={frame!r}" if frame else f"could not read header.frame_id from {sensor_endpoint.name}",
                required=False,
            )
        )


def _read_topic_field_once(topic_name: str, field: str, timeout: float = 6.0) -> str | None:
    command = ["ros2", "topic", "echo", "--once", topic_name, "--field", field]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    output = completed.stdout.strip()
    return output or None


def _add_tf_checks(checks: list[ValidationCheck]) -> None:
    map_to_odom = _tf_echo_available("map", "odom")
    checks.append(
        ValidationCheck(
            name="tf_map_to_odom",
            status="pass" if map_to_odom else "fail",
            detail="map -> odom available" if map_to_odom else "map -> odom transform unavailable",
            required=True,
        )
    )

    base_frame = None
    for candidate in ["base_link", "base_footprint"]:
        if _tf_echo_available("odom", candidate):
            base_frame = candidate
            break
    checks.append(
        ValidationCheck(
            name="tf_odom_to_base",
            status="pass" if base_frame else "fail",
            detail=f"odom -> {base_frame} available" if base_frame else "odom -> base_link/base_footprint transform unavailable",
            required=True,
        )
    )


def _tf_echo_available(target_frame: str, source_frame: str, timeout: float = 5.0) -> bool:
    command = ["ros2", "run", "tf2_ros", "tf2_echo", target_frame, source_frame]
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        output = "\n".join([completed.stdout, completed.stderr])
        return _tf_echo_output_has_transform(output)
    except FileNotFoundError:
        return False
    except subprocess.TimeoutExpired as exc:
        output = "\n".join([(exc.stdout or ""), (exc.stderr or "")])
        return _tf_echo_output_has_transform(output)


def _tf_echo_output_has_transform(output: str) -> bool:
    normalized = output or ""
    return "At time " in normalized or "Translation:" in normalized or "Rotation:" in normalized


def _action_check(
    actions: dict[str, ResourceEndpoint],
    action_name: str,
    expected_types: set[str],
) -> ValidationCheck:
    endpoint = actions.get(_normalize_name(action_name))
    if endpoint is None:
        return ValidationCheck(
            name="action_navigate_to_pose",
            status="fail",
            detail=f"{action_name} action is missing",
            required=True,
        )
    if endpoint.msg_type and endpoint.msg_type not in expected_types:
        return ValidationCheck(
            name="action_navigate_to_pose",
            status="fail",
            detail=f"{endpoint.name} has type {endpoint.msg_type}, expected one of {sorted(expected_types)}",
            required=True,
        )
    return ValidationCheck(
        name="action_navigate_to_pose",
        status="pass",
        detail=f"{endpoint.name}" + (f" [{endpoint.msg_type}]" if endpoint.msg_type else ""),
        required=True,
    )
