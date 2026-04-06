from __future__ import annotations

import copy
import json
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .topics import TOPIC_HINTS


DEFAULT_RESOURCE_HINTS = {
    **TOPIC_HINTS,
    "imu": ["/imu", "imu", "/imu/data", "imu/data"],
    "gps_fix": ["/fix", "fix", "/gps/fix", "gps/fix", "/gps/filtered"],
}

DEFAULT_EXPECTED_TOPIC_TYPES = {
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

BUILTIN_PROFILE_SPECS = {
    "vehicle": {
        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel"],
        "recommended_roles": ["goal_pose", "imu", "gps_fix"],
        "required_sensor_roles": ["scan", "pointcloud"],
        "required_actions": [],
        "host_checks": [],
        "frame_checks": [
            {"name": "map_frame", "role": "map", "field": "header.frame_id", "equals": "map", "required": True},
            {"name": "odom_frame", "role": "odom", "field": "header.frame_id", "equals": "odom", "required": True},
            {
                "name": "base_frame",
                "role": "odom",
                "field": "child_frame_id",
                "equals_any": ["base_link", "base_footprint"],
                "required": True,
            },
            {
                "name": "sensor_frame",
                "roles": ["scan", "pointcloud"],
                "field": "header.frame_id",
                "nonempty": True,
                "required": False,
            },
        ],
        "required_transforms": [
            {"name": "tf_map_to_odom", "target": "map", "source": "odom", "required": True},
            {
                "name": "tf_odom_to_base",
                "pairs": [
                    {"target": "odom", "source": "base_link"},
                    {"target": "odom", "source": "base_footprint"},
                ],
                "required": True,
            },
        ],
    },
    "nav2": {
        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel"],
        "recommended_roles": ["goal_pose", "imu"],
        "required_sensor_roles": ["scan", "pointcloud"],
        "required_actions": [
            {
                "name": "/navigate_to_pose",
                "check_name": "action_navigate_to_pose",
                "types": ["nav2_msgs/action/NavigateToPose"],
                "required": True,
            }
        ],
        "host_checks": [],
        "frame_checks": [
            {"name": "map_frame", "role": "map", "field": "header.frame_id", "equals": "map", "required": True},
            {"name": "odom_frame", "role": "odom", "field": "header.frame_id", "equals": "odom", "required": True},
            {
                "name": "base_frame",
                "role": "odom",
                "field": "child_frame_id",
                "equals_any": ["base_link", "base_footprint"],
                "required": True,
            },
            {
                "name": "sensor_frame",
                "roles": ["scan", "pointcloud"],
                "field": "header.frame_id",
                "nonempty": True,
                "required": False,
            },
        ],
        "required_transforms": [
            {"name": "tf_map_to_odom", "target": "map", "source": "odom", "required": True},
            {
                "name": "tf_odom_to_base",
                "pairs": [
                    {"target": "odom", "source": "base_link"},
                    {"target": "odom", "source": "base_footprint"},
                ],
                "required": True,
            },
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
    profile_file: Path | None = None,
    repo_root: Path | None = None,
) -> ValidationReport:
    repo_root = (repo_root or Path.cwd()).resolve()
    profile_name, spec, resource_hints, expected_topic_types = _load_validation_config(
        profile=profile,
        profile_file=profile_file,
        repo_root=repo_root,
    )

    endpoints = load_topic_snapshot(topics=topics, topic_file=topic_file)
    checks: list[ValidationCheck] = []
    warnings: list[str] = []
    live_probe = topics is None and topic_file is None

    for role in spec["required_roles"]:
        checks.append(
            _topic_role_check(
                role,
                endpoints,
                resource_hints=resource_hints,
                expected_topic_types=expected_topic_types,
                required=True,
            )
        )
    for role in spec["recommended_roles"]:
        checks.append(
            _topic_role_check(
                role,
                endpoints,
                resource_hints=resource_hints,
                expected_topic_types=expected_topic_types,
                required=False,
            )
        )
    checks.append(
        _sensor_check(
            endpoints,
            roles=spec["required_sensor_roles"],
            resource_hints=resource_hints,
            expected_topic_types=expected_topic_types,
            required=True,
        )
    )

    if live_probe:
        _add_frame_checks(checks, endpoints, spec["frame_checks"], resource_hints)
        _add_tf_checks(checks, spec["required_transforms"])
        if spec["required_actions"]:
            actions = _load_action_snapshot()
            for action_spec in spec["required_actions"]:
                checks.append(_action_check(actions, action_spec))
        if spec["host_checks"]:
            _add_host_checks(checks, spec["host_checks"])
    else:
        warnings.append(
            "Live frame, TF, action, and host probes were skipped because a topic snapshot was supplied."
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


def _load_validation_config(
    profile: str,
    profile_file: Path | None,
    repo_root: Path,
) -> tuple[str, dict[str, Any], dict[str, list[str]], dict[str, set[str]]]:
    if profile_file is not None:
        payload = _load_profile_json(_resolve_profile_file(profile_file, repo_root))
        return _materialize_custom_profile(payload, default_name=profile, repo_root=repo_root)

    profile_name = profile.lower()
    if profile_name in BUILTIN_PROFILE_SPECS:
        return (
            profile_name,
            copy.deepcopy(BUILTIN_PROFILE_SPECS[profile_name]),
            copy.deepcopy(DEFAULT_RESOURCE_HINTS),
            copy.deepcopy(DEFAULT_EXPECTED_TOPIC_TYPES),
        )

    discovered = _find_named_profile_file(profile_name, repo_root)
    if discovered is not None:
        payload = _load_profile_json(discovered)
        return _materialize_custom_profile(payload, default_name=profile_name, repo_root=repo_root)

    known = ", ".join(sorted(BUILTIN_PROFILE_SPECS))
    raise ValueError(
        f"Unknown validation profile: {profile}. Use one of [{known}], pass --profile-file, "
        f"or create profiles/{profile_name}.json."
    )


def _resolve_profile_file(profile_file: Path, repo_root: Path) -> Path:
    candidate = profile_file if profile_file.is_absolute() else repo_root / profile_file
    if not candidate.exists():
        raise ValueError(f"Validation profile file does not exist: {candidate}")
    return candidate.resolve()


def _find_named_profile_file(profile_name: str, repo_root: Path) -> Path | None:
    for candidate in [
        repo_root / "profiles" / f"{profile_name}.json",
        repo_root / ".nav2pp" / "profiles" / f"{profile_name}.json",
    ]:
        if candidate.exists():
            return candidate.resolve()
    return None


def _load_profile_json(profile_path: Path) -> dict[str, Any]:
    payload = json.loads(profile_path.read_text())
    if not isinstance(payload, dict):
        raise ValueError(f"Validation profile {profile_path} must be a JSON object.")
    return payload


def _materialize_custom_profile(
    payload: dict[str, Any],
    default_name: str,
    repo_root: Path,
) -> tuple[str, dict[str, Any], dict[str, list[str]], dict[str, set[str]]]:
    extends = payload.get("extends")
    if extends is None:
        spec = _empty_profile_spec()
        resource_hints = copy.deepcopy(DEFAULT_RESOURCE_HINTS)
        expected_topic_types = copy.deepcopy(DEFAULT_EXPECTED_TOPIC_TYPES)
    else:
        if not isinstance(extends, str):
            raise ValueError("Validation profile field 'extends' must be a string.")
        if extends.lower() == default_name.lower():
            raise ValueError(f"Validation profile {default_name} cannot extend itself.")
        _, spec, resource_hints, expected_topic_types = _load_validation_config(
            profile=extends,
            profile_file=None,
            repo_root=repo_root,
        )

    if "resource_hints" in payload:
        resource_hints.update(_normalize_hint_map(payload["resource_hints"], "resource_hints"))
    if "expected_topic_types" in payload:
        expected_topic_types.update(
            _normalize_type_map(payload["expected_topic_types"], "expected_topic_types")
        )

    for field_name in [
        "required_roles",
        "recommended_roles",
        "required_sensor_roles",
    ]:
        if field_name in payload:
            spec[field_name] = _normalize_string_list(payload[field_name], field_name)

    if "required_actions" in payload:
        spec["required_actions"] = _normalize_action_specs(payload["required_actions"])
    if "host_checks" in payload:
        spec["host_checks"] = _normalize_host_checks(payload["host_checks"])
    if "frame_checks" in payload:
        spec["frame_checks"] = _normalize_frame_checks(payload["frame_checks"])
    if "required_transforms" in payload:
        spec["required_transforms"] = _normalize_transform_specs(payload["required_transforms"])

    profile_name = str(payload.get("name", default_name)).strip() or default_name
    return profile_name, spec, resource_hints, expected_topic_types


def _empty_profile_spec() -> dict[str, Any]:
    return {
        "required_roles": [],
        "recommended_roles": [],
        "required_sensor_roles": [],
        "required_actions": [],
        "host_checks": [],
        "frame_checks": [],
        "required_transforms": [],
    }


def _normalize_string_list(raw: Any, field_name: str) -> list[str]:
    if not isinstance(raw, list) or not all(isinstance(item, str) and item.strip() for item in raw):
        raise ValueError(f"Validation profile field '{field_name}' must be a list of strings.")
    return [item.strip() for item in raw]


def _normalize_hint_map(raw: Any, field_name: str) -> dict[str, list[str]]:
    if not isinstance(raw, dict):
        raise ValueError(f"Validation profile field '{field_name}' must be a JSON object.")
    normalized: dict[str, list[str]] = {}
    for key, value in raw.items():
        if not isinstance(key, str) or not key.strip():
            raise ValueError(f"Validation profile field '{field_name}' contains an empty role name.")
        if isinstance(value, str):
            hints = [value]
        elif isinstance(value, list) and all(isinstance(item, str) and item.strip() for item in value):
            hints = [item.strip() for item in value]
        else:
            raise ValueError(
                f"Validation profile field '{field_name}.{key}' must be a string or list of strings."
            )
        normalized[key.strip()] = hints
    return normalized


def _normalize_type_map(raw: Any, field_name: str) -> dict[str, set[str]]:
    if not isinstance(raw, dict):
        raise ValueError(f"Validation profile field '{field_name}' must be a JSON object.")
    normalized: dict[str, set[str]] = {}
    for key, value in raw.items():
        if not isinstance(key, str) or not key.strip():
            raise ValueError(f"Validation profile field '{field_name}' contains an empty role name.")
        if isinstance(value, str):
            types = {value.strip()}
        elif isinstance(value, list) and all(isinstance(item, str) and item.strip() for item in value):
            types = {item.strip() for item in value}
        else:
            raise ValueError(
                f"Validation profile field '{field_name}.{key}' must be a string or list of strings."
            )
        normalized[key.strip()] = types
    return normalized


def _normalize_action_specs(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        raise ValueError("Validation profile field 'required_actions' must be a list.")
    normalized: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("Each validation action spec must be a JSON object.")
        action_name = item.get("name")
        if not isinstance(action_name, str) or not action_name.strip():
            raise ValueError("Each validation action spec must include a non-empty 'name'.")
        types = item.get("types", [])
        if isinstance(types, str):
            type_list = [types.strip()]
        elif isinstance(types, list) and all(isinstance(entry, str) and entry.strip() for entry in types):
            type_list = [entry.strip() for entry in types]
        else:
            raise ValueError(f"Validation action {action_name} must define 'types' as a string or list.")
        normalized.append(
            {
                "name": action_name.strip(),
                "check_name": str(item.get("check_name") or f"action_{_slug(action_name)}"),
                "types": set(type_list),
                "required": bool(item.get("required", True)),
            }
        )
    return normalized


def _normalize_host_checks(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        raise ValueError("Validation profile field 'host_checks' must be a list.")
    normalized: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("Each host check must be a JSON object.")
        name = item.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Each host check must include a non-empty 'name'.")
        command = item.get("command")
        if isinstance(command, str):
            command_list = [command.strip()]
        elif isinstance(command, list) and all(isinstance(part, str) and part.strip() for part in command):
            command_list = [part.strip() for part in command]
        else:
            raise ValueError(f"Host check {name} must define 'command' as a string or list of strings.")
        normalized.append(
            {
                "name": name.strip(),
                "command": command_list,
                "required": bool(item.get("required", True)),
                "nonempty_stdout": bool(item.get("nonempty_stdout", False)),
                "contains_stdout": (
                    str(item["contains_stdout"]).strip()
                    if "contains_stdout" in item and str(item["contains_stdout"]).strip()
                    else None
                ),
            }
        )
    return normalized


def _normalize_frame_checks(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        raise ValueError("Validation profile field 'frame_checks' must be a list.")
    normalized: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("Each frame check must be a JSON object.")
        field_name = item.get("field")
        if not isinstance(field_name, str) or not field_name.strip():
            raise ValueError("Each frame check must include a non-empty 'field'.")
        spec: dict[str, Any] = {
            "name": str(item.get("name") or _slug(field_name)),
            "field": field_name.strip(),
            "required": bool(item.get("required", True)),
        }
        if "topic" in item and isinstance(item["topic"], str) and item["topic"].strip():
            spec["topic"] = item["topic"].strip()
        elif "role" in item and isinstance(item["role"], str) and item["role"].strip():
            spec["role"] = item["role"].strip()
        elif "roles" in item:
            spec["roles"] = _normalize_string_list(item["roles"], "frame_checks.roles")
        else:
            raise ValueError("Each frame check must define one of: 'topic', 'role', or 'roles'.")

        if "equals" in item:
            spec["equals"] = str(item["equals"])
        if "equals_any" in item:
            spec["equals_any"] = _normalize_string_list(item["equals_any"], "frame_checks.equals_any")
        if item.get("nonempty"):
            spec["nonempty"] = True
        if not any(key in spec for key in ["equals", "equals_any", "nonempty"]):
            spec["nonempty"] = True
        normalized.append(spec)
    return normalized


def _normalize_transform_specs(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        raise ValueError("Validation profile field 'required_transforms' must be a list.")
    normalized: list[dict[str, Any]] = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("Each transform check must be a JSON object.")
        spec: dict[str, Any] = {
            "name": str(item.get("name") or "tf_check"),
            "required": bool(item.get("required", True)),
        }
        if "pairs" in item:
            pairs = item["pairs"]
            if not isinstance(pairs, list) or not pairs:
                raise ValueError("Transform checks with 'pairs' must provide a non-empty list.")
            normalized_pairs: list[dict[str, str]] = []
            for pair in pairs:
                if not isinstance(pair, dict):
                    raise ValueError("Each transform pair must be a JSON object.")
                target = pair.get("target")
                source = pair.get("source")
                if not isinstance(target, str) or not isinstance(source, str) or not target.strip() or not source.strip():
                    raise ValueError("Each transform pair must include non-empty 'target' and 'source'.")
                normalized_pairs.append({"target": target.strip(), "source": source.strip()})
            spec["pairs"] = normalized_pairs
        else:
            target = item.get("target")
            source = item.get("source")
            if not isinstance(target, str) or not isinstance(source, str) or not target.strip() or not source.strip():
                raise ValueError("Each transform check must include non-empty 'target' and 'source'.")
            spec["target"] = target.strip()
            spec["source"] = source.strip()
        normalized.append(spec)
    return normalized


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
    resource_hints: dict[str, list[str]],
    expected_topic_types: dict[str, set[str]],
    required: bool,
) -> ValidationCheck:
    endpoint = _resolve_endpoint(endpoints, resource_hints.get(role, []))
    if endpoint is None:
        return ValidationCheck(
            name=role,
            status="fail" if required else "warn",
            detail="missing from the ROS graph",
            required=required,
        )

    expected_types = expected_topic_types.get(role, set())
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
    resource_hints: dict[str, list[str]],
    expected_topic_types: dict[str, set[str]],
    required: bool,
) -> ValidationCheck:
    resolved = [
        (role, _resolve_endpoint(endpoints, resource_hints.get(role, [])))
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
        expected_types = expected_topic_types.get(role, set())
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


def _add_frame_checks(
    checks: list[ValidationCheck],
    endpoints: dict[str, ResourceEndpoint],
    frame_specs: list[dict[str, Any]],
    resource_hints: dict[str, list[str]],
) -> None:
    for frame_spec in frame_specs:
        endpoint = _resolve_endpoint_for_frame_spec(endpoints, frame_spec, resource_hints)
        required = bool(frame_spec.get("required", True))
        if endpoint is None:
            checks.append(
                ValidationCheck(
                    name=frame_spec["name"],
                    status="fail" if required else "warn",
                    detail="no matching topic was found for the frame check",
                    required=required,
                )
            )
            continue

        value = _read_topic_field_once(endpoint.name, frame_spec["field"])
        status = _evaluate_frame_value(frame_spec, value, required)
        checks.append(
            ValidationCheck(
                name=frame_spec["name"],
                status=status,
                detail=(
                    f"{endpoint.name} {frame_spec['field']}={value!r}"
                    if value is not None
                    else f"could not read {frame_spec['field']} from {endpoint.name}"
                ),
                required=required,
            )
        )


def _resolve_endpoint_for_frame_spec(
    endpoints: dict[str, ResourceEndpoint],
    frame_spec: dict[str, Any],
    resource_hints: dict[str, list[str]],
) -> ResourceEndpoint | None:
    if "topic" in frame_spec:
        return _resolve_endpoint(endpoints, [frame_spec["topic"]])
    if "role" in frame_spec:
        return _resolve_endpoint(endpoints, resource_hints.get(frame_spec["role"], []))
    for role in frame_spec.get("roles", []):
        endpoint = _resolve_endpoint(endpoints, resource_hints.get(role, []))
        if endpoint is not None:
            return endpoint
    return None


def _evaluate_frame_value(frame_spec: dict[str, Any], value: str | None, required: bool) -> str:
    if value is None:
        return "fail" if required else "warn"
    if "equals" in frame_spec:
        expected = frame_spec["equals"]
        return "pass" if value == expected else ("fail" if required else "warn")
    if "equals_any" in frame_spec:
        expected_values = set(frame_spec["equals_any"])
        return "pass" if value in expected_values else ("fail" if required else "warn")
    if frame_spec.get("nonempty"):
        return "pass" if value else ("fail" if required else "warn")
    return "pass"


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


def _add_tf_checks(checks: list[ValidationCheck], transform_specs: list[dict[str, Any]]) -> None:
    for transform_spec in transform_specs:
        required = bool(transform_spec.get("required", True))
        pairs = transform_spec.get("pairs")
        if pairs:
            matched = None
            for pair in pairs:
                if _tf_echo_available(pair["target"], pair["source"]):
                    matched = pair
                    break
            checks.append(
                ValidationCheck(
                    name=transform_spec["name"],
                    status="pass" if matched else ("fail" if required else "warn"),
                    detail=(
                        f"{matched['target']} -> {matched['source']} available"
                        if matched
                        else _render_transform_pair_failure(pairs)
                    ),
                    required=required,
                )
            )
            continue

        target = transform_spec["target"]
        source = transform_spec["source"]
        available = _tf_echo_available(target, source)
        checks.append(
            ValidationCheck(
                name=transform_spec["name"],
                status="pass" if available else ("fail" if required else "warn"),
                detail=f"{target} -> {source} available" if available else f"{target} -> {source} transform unavailable",
                required=required,
            )
        )


def _render_transform_pair_failure(pairs: list[dict[str, str]]) -> str:
    options = ", ".join(f"{pair['target']} -> {pair['source']}" for pair in pairs)
    return f"none of the expected transforms are available: {options}"


def _add_host_checks(checks: list[ValidationCheck], host_specs: list[dict[str, Any]]) -> None:
    for host_spec in host_specs:
        checks.append(_host_check(host_spec))


def _host_check(host_spec: dict[str, Any]) -> ValidationCheck:
    required = bool(host_spec.get("required", True))
    completed = _run_host_command(host_spec["command"])
    if completed is None:
        return ValidationCheck(
            name=host_spec["name"],
            status="fail" if required else "warn",
            detail=f"command not found: {host_spec['command'][0]}",
            required=required,
        )

    stdout = (completed.stdout or "").strip()
    stderr = (completed.stderr or "").strip()
    if completed.returncode != 0:
        detail = stderr or stdout or f"command failed with exit code {completed.returncode}"
        return ValidationCheck(
            name=host_spec["name"],
            status="fail" if required else "warn",
            detail=detail,
            required=required,
        )

    if host_spec.get("nonempty_stdout") and not stdout:
        return ValidationCheck(
            name=host_spec["name"],
            status="fail" if required else "warn",
            detail="command succeeded but returned empty stdout",
            required=required,
        )

    contains_stdout = host_spec.get("contains_stdout")
    if contains_stdout and contains_stdout not in stdout:
        return ValidationCheck(
            name=host_spec["name"],
            status="fail" if required else "warn",
            detail=f"stdout did not contain {contains_stdout!r}",
            required=required,
        )

    detail = stdout.splitlines()[0] if stdout else "command available"
    return ValidationCheck(
        name=host_spec["name"],
        status="pass",
        detail=detail,
        required=required,
    )


def _run_host_command(command: list[str], timeout: float = 8.0) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except FileNotFoundError:
        return None
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            command,
            returncode=124,
            stdout=(exc.stdout or ""),
            stderr=(exc.stderr or "command timed out"),
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


def _action_check(actions: dict[str, ResourceEndpoint], action_spec: dict[str, Any]) -> ValidationCheck:
    action_name = _normalize_name(action_spec["name"])
    endpoint = actions.get(action_name)
    required = bool(action_spec.get("required", True))
    if endpoint is None:
        return ValidationCheck(
            name=action_spec["check_name"],
            status="fail" if required else "warn",
            detail=f"{action_name} action is missing",
            required=required,
        )
    expected_types = action_spec.get("types", set())
    if expected_types and endpoint.msg_type and endpoint.msg_type not in expected_types:
        return ValidationCheck(
            name=action_spec["check_name"],
            status="fail" if required else "warn",
            detail=f"{endpoint.name} has type {endpoint.msg_type}, expected one of {sorted(expected_types)}",
            required=required,
        )
    return ValidationCheck(
        name=action_spec["check_name"],
        status="pass",
        detail=f"{endpoint.name}" + (f" [{endpoint.msg_type}]" if endpoint.msg_type else ""),
        required=required,
    )


def _slug(value: str) -> str:
    return value.strip().lower().replace("/", "_").replace(" ", "_").strip("_")
