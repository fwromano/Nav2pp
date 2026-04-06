from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass
from pathlib import Path


TOPIC_HINTS = {
    "map": ["/map", "map"],
    "odom": ["/odom", "odom", "/odometry/filtered", "wheel/odometry"],
    "scan": ["/scan", "scan", "/lidar/scan", "lidar/scan", "/laser_scan"],
    "pointcloud": ["/points", "/pointcloud", "/velodyne_points"],
    "cmd_vel": ["/cmd_vel", "cmd_vel", "/cmd_vel_nav", "/nav/cmd_vel"],
    "tf": ["/tf", "tf"],
    "tf_static": ["/tf_static", "tf_static"],
    "goal_pose": ["/goal_pose", "/move_base_simple/goal"],
}


@dataclass
class TopicMatch:
    topic_type: str
    resolved_name: str | None
    confidence: str


def discover_topics(
    topics: list[str] | None = None,
    topic_file: Path | None = None,
) -> dict[str, TopicMatch]:
    available = _load_topics(topics=topics, topic_file=topic_file)
    matches: dict[str, TopicMatch] = {}
    for topic_type, hints in TOPIC_HINTS.items():
        resolved = _resolve_topic(available, hints)
        matches[topic_type] = TopicMatch(
            topic_type=topic_type,
            resolved_name=resolved,
            confidence="high" if resolved else "missing",
        )
    return matches


def render_topic_report(matches: dict[str, TopicMatch], as_json: bool) -> str:
    if as_json:
        payload = {
            topic_type: {
                "resolved_name": match.resolved_name,
                "confidence": match.confidence,
            }
            for topic_type, match in matches.items()
        }
        return json.dumps(payload, indent=2, sort_keys=True)

    lines = []
    for topic_type, match in matches.items():
        resolved = match.resolved_name or "missing"
        lines.append(f"{topic_type:10s} {resolved:30s} {match.confidence}")
    return "\n".join(lines)


def _load_topics(topics: list[str] | None, topic_file: Path | None) -> list[str]:
    if topics is not None:
        return _normalize_topics(topics)
    if topic_file:
        return _normalize_topics(topic_file.read_text().splitlines())
    ros2_topics = _run_ros2_topic_list()
    return _normalize_topics(ros2_topics)


def _run_ros2_topic_list() -> list[str]:
    try:
        completed = subprocess.run(
            ["ros2", "topic", "list"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("ros2 is not installed or not on PATH") from exc
    if completed.returncode != 0:
        stderr = completed.stderr.strip() or "ros2 topic list failed"
        raise RuntimeError(stderr)
    return completed.stdout.splitlines()


def _normalize_topics(raw_topics: list[str]) -> list[str]:
    normalized = []
    for topic in raw_topics:
        stripped = topic.strip()
        if not stripped:
            continue
        if not stripped.startswith("/"):
            stripped = f"/{stripped}"
        normalized.append(stripped)
    return normalized


def _resolve_topic(available: list[str], hints: list[str]) -> str | None:
    available_set = set(available)
    for hint in hints:
        candidate = hint if hint.startswith("/") else f"/{hint}"
        if candidate in available_set:
            return candidate

    for hint in hints:
        normalized = hint.lstrip("/")
        for topic in available:
            if topic.lstrip("/").endswith(normalized):
                return topic
    return None
