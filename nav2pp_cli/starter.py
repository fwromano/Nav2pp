from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .bootstrap import _ensure_local_venv, _ensure_repo_layout
from .diagnostics import HostInfo, collect_host_info
from .topics import TOPIC_HINTS, discover_topics


REQUIRED_INPUTS = ["map", "odom", "tf", "tf_static"]
SENSOR_INPUTS = ["scan", "pointcloud"]


@dataclass
class StartPlan:
    strategy: str
    platform_label: str
    summary: str
    matched_topics: dict[str, str | None]
    stubbed_topics: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    rationale: list[str] = field(default_factory=list)
    runner_script: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def run_start(
    repo_root: Path,
    mode: str,
    topics: list[str] | None,
    topic_file: Path | None,
    dry_run: bool,
    yes: bool,
) -> tuple[HostInfo, StartPlan]:
    repo_root = repo_root.resolve()
    host = collect_host_info(repo_root)
    _ensure_repo_layout(repo_root)
    _ensure_local_venv(repo_root)

    setup_strategy = _load_selected_setup_strategy(repo_root)
    matches, detection_warning = _safe_discover_topics(topics=topics, topic_file=topic_file)
    plan = build_start_plan(
        host,
        repo_root,
        matches,
        mode=mode,
        detection_warning=detection_warning,
        setup_strategy=setup_strategy,
    )
    _write_start_state(repo_root, host, plan)
    _write_start_scripts(repo_root, host, plan)

    if dry_run:
        return host, plan

    runner = repo_root / ".nav2pp" / "generated" / plan.runner_script
    if not yes and sys.stdin.isatty():
        if not _confirm(f"Run generated start script now ({runner.name})? [Y/n] "):
            return host, plan
    elif not yes and not sys.stdin.isatty():
        print(
            "Skipping execution because this session is non-interactive. Re-run with --yes to execute the start script.",
            file=sys.stderr,
        )
        return host, plan

    subprocess.run([str(runner)], check=True)
    return host, plan


def build_start_plan(
    host: HostInfo,
    repo_root: Path,
    matches: dict[str, Any],
    mode: str = "auto",
    detection_warning: str | None = None,
    setup_strategy: str | None = None,
) -> StartPlan:
    requested_mode = mode.lower()
    matched_topics = {
        topic_type: getattr(match, "resolved_name", None)
        for topic_type, match in matches.items()
    }
    warnings: list[str] = []
    if detection_warning:
        if host.system == "Darwin" and detection_warning == "ros2 is not installed or not on PATH":
            if setup_strategy == "macos-docker":
                warnings.append("Host ros2 is not on PATH. nav2++ will use ROS inside the Docker runtime.")
            else:
                warnings.append("Host ros2 is not on PATH. nav2++ will use ROS inside the Lima guest.")
        else:
            warnings.append(detection_warning)

    sensor_available = any(matched_topics.get(name) for name in SENSOR_INPUTS)
    live_signals = sum(
        1
        for name in ["odom", "tf", "tf_static", "scan", "pointcloud"]
        if matched_topics.get(name)
    )

    if requested_mode == "sim":
        strategy = "tb3-demo"
    elif requested_mode == "live":
        strategy = "live-rviz-overlay"
    elif live_signals >= 2 and sensor_available:
        strategy = "live-rviz-overlay"
    else:
        strategy = "tb3-demo"

    stubbed_topics: list[str] = []
    if strategy == "live-rviz-overlay":
        for topic_name in REQUIRED_INPUTS:
            if not matched_topics.get(topic_name):
                stubbed_topics.append(topic_name)
        if not sensor_available:
            stubbed_topics.append("scan")

    if host.system == "Darwin" and setup_strategy == "macos-docker" and strategy == "live-rviz-overlay":
        warnings.append("Docker runtime on macOS currently supports the packaged demo path only. Falling back to tb3-demo.")
        strategy = "tb3-demo"
        stubbed_topics = []

    platform_label = f"{host.system} {host.machine}"
    if host.hw_model:
        platform_label = f"{platform_label} ({host.hw_model})"

    if host.system == "Darwin":
        if setup_strategy == "macos-docker":
            runner_script = "start-macos-docker.sh"
            if strategy == "tb3-demo":
                summary = "Launch the official Nav2 TB3 simulation inside a Docker desktop container and expose RViz/Gazebo at a local browser URL."
                rationale = [
                    "The packaged TB3 demo is the straightest path to a visible Nav2 session.",
                    "The browser desktop avoids macOS guest-display forwarding issues.",
                ]
            else:
                summary = "Launch RViz inside the Docker desktop container against a live or stubbed graph."
                rationale = [
                    "A browser-served desktop keeps the GUI path predictable on macOS.",
                    "The container keeps ROS and system packages isolated from the host.",
                ]
        else:
            runner_script = "start-macos-lima.sh"
            if strategy == "tb3-demo":
                summary = (
                    "Enter the Lima guest and launch the official Nav2 TB3 simulation, "
                    "using Gazebo and RViz when a guest display is available and headless mode otherwise."
                )
                rationale = [
                    "Nav2's documented quick-start path is the TB3 simulation launch.",
                    "On macOS, the practical place to run it is inside the Linux guest created by setup.",
                ]
            else:
                summary = "Enter the Lima guest, stub missing core topics, and launch RViz against the live or stubbed graph."
                rationale = [
                    "A partial live graph should still be explorable if the missing basics are stubbed.",
                    "The guest keeps ROS tooling out of the macOS host.",
                ]
            if not str(repo_root).startswith(str(Path.home())):
                warnings.append(
                    "Repo is outside $HOME. The Lima guest will use $HOME/nav2pp for its local state and start scripts."
                )
    else:
        runner_script = "start-linux.sh"
        if strategy == "tb3-demo":
            summary = "Launch the official Nav2 TB3 simulation with Gazebo and RViz."
            rationale = [
                "This is the straightest path to a working Nav2 demo in one command after setup.",
                "It matches the documented Nav2 getting-started example.",
            ]
        else:
            summary = "Stub missing core topics and launch RViz against the current ROS graph."
            rationale = [
                "If part of the robot graph already exists, it is more useful to overlay missing basics than to ignore it.",
                "This gives you a GUI and a stable frame tree without forcing a full simulator.",
            ]

    return StartPlan(
        strategy=strategy,
        platform_label=platform_label,
        summary=summary,
        matched_topics=matched_topics,
        stubbed_topics=stubbed_topics,
        warnings=warnings,
        rationale=rationale,
        runner_script=runner_script,
    )


def _safe_discover_topics(
    topics: list[str] | None,
    topic_file: Path | None,
) -> tuple[dict[str, Any], str | None]:
    try:
        return discover_topics(topics=topics, topic_file=topic_file), None
    except RuntimeError as exc:
        return discover_topics(topics=[], topic_file=None), str(exc)


def _load_selected_setup_strategy(repo_root: Path) -> str | None:
    path = repo_root / ".nav2pp" / "state" / "plan.json"
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return None
    strategy = payload.get("strategy")
    return strategy if isinstance(strategy, str) else None


def _write_start_state(repo_root: Path, host: HostInfo, plan: StartPlan) -> None:
    state_dir = repo_root / ".nav2pp" / "state"
    (state_dir / "host.json").write_text(host.to_json() + "\n")
    (state_dir / "start-plan.json").write_text(json.dumps(plan.to_dict(), indent=2, sort_keys=True) + "\n")


def _write_start_scripts(repo_root: Path, host: HostInfo, plan: StartPlan) -> None:
    generated_dir = repo_root / ".nav2pp" / "generated"
    if host.system == "Darwin":
        if plan.runner_script == "start-macos-docker.sh":
            script = _render_macos_docker_start_script(repo_root, plan)
            path = generated_dir / "start-macos-docker.sh"
        else:
            script = _render_macos_start_script(repo_root, plan)
            path = generated_dir / "start-macos-lima.sh"
    else:
        script = _render_linux_start_script(repo_root, plan)
        path = generated_dir / "start-linux.sh"
    path.write_text(script)
    path.chmod(0o755)


def _render_linux_start_script(repo_root: Path, plan: StartPlan) -> str:
    body = _render_linux_demo_body(repo_root) if plan.strategy == "tb3-demo" else _render_live_overlay_body(repo_root, plan)
    return "\n".join(
        [
            "#!/usr/bin/env bash",
            "set -euo pipefail",
            f'REPO_ROOT="{repo_root}"',
            body.rstrip(),
            "",
        ]
    )


def _render_macos_start_script(repo_root: Path, plan: StartPlan) -> str:
    repo_mountable = str(repo_root).startswith(str(Path.home()))
    guest_repo_root = str(repo_root) if repo_mountable else "$HOME/nav2pp"
    guest_body = _render_guest_demo_body() if plan.strategy == "tb3-demo" else _render_guest_live_overlay_body(plan)
    header_lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        f'PREFERRED_GUEST_REPO_ROOT="{guest_repo_root}"',
        "",
        "if ! command -v limactl >/dev/null 2>&1; then",
        '  echo "limactl is not installed. Run ./nav2++ setup --yes first." >&2',
        "  exit 1",
        "fi",
        "",
        "if ! limactl list | awk '{print $1}' | grep -qx nav2pp; then",
        '  echo "Lima VM nav2pp does not exist yet. Run ./nav2++ setup --yes first." >&2',
        "  exit 1",
        "fi",
        "",
        "limactl shell nav2pp -- env PREFERRED_GUEST_REPO_ROOT=\"${PREFERRED_GUEST_REPO_ROOT}\" bash -s <<'EOF'",
    ]
    return "\n".join(header_lines) + "\n" + guest_body.rstrip() + "\nEOF\n"


def _render_macos_docker_start_script(repo_root: Path, plan: StartPlan) -> str:
    url = "http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale"
    return "\n".join(
        [
            "#!/usr/bin/env bash",
            "set -euo pipefail",
            f'REPO_ROOT="{repo_root}"',
            'IMAGE_TAG="nav2pp-jazzy-desktop:local"',
            'CONTAINER_NAME="nav2pp-desktop"',
            f'BROWSER_URL="{url}"',
            "",
            "if ! command -v docker >/dev/null 2>&1; then",
            '  echo "docker is not installed. Run ./nav2++ setup --mode docker --yes first." >&2',
            "  exit 1",
            "fi",
            "",
            "if ! docker info >/dev/null 2>&1; then",
            "  if command -v colima >/dev/null 2>&1; then",
            "    colima start --cpu 6 --memory 12 --disk 80",
            "  fi",
            "fi",
            "",
            "if ! docker info >/dev/null 2>&1; then",
            '  echo "Docker engine is unavailable. Run ./nav2++ setup --mode docker --yes first." >&2',
            "  exit 1",
            "fi",
            "",
            'docker build -t "${IMAGE_TAG}" -f "${REPO_ROOT}/docker/nav2pp-desktop/Dockerfile" "${REPO_ROOT}"',
            "",
            'docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true',
            'echo "Nav2++ browser UI: ${BROWSER_URL}"',
            'if command -v open >/dev/null 2>&1; then',
            '  (sleep 5; open "${BROWSER_URL}" >/dev/null 2>&1 || true) &',
            "fi",
            "",
            'exec docker run --rm --name "${CONTAINER_NAME}" --shm-size=1g -p 6080:6080 -v "${REPO_ROOT}:/work/Nav2++" -e NAV2PP_REPO_ROOT=/work/Nav2++ "${IMAGE_TAG}"',
            "",
        ]
    )


def _render_linux_demo_body(repo_root: Path) -> str:
    fallback_body = textwrap.indent(_render_live_overlay_body(repo_root, _fallback_live_plan()), "  ")
    return textwrap.dedent(
        f"""\
        source "$REPO_ROOT/.nav2pp/env/nav2pp.env"
        export TURTLEBOT3_MODEL=waffle

        if ! command -v ros2 >/dev/null 2>&1; then
          echo "ros2 is not on PATH. Run ./nav2++ setup first and source .nav2pp/env/nav2pp.env." >&2
          exit 1
        fi

        if ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
          exec ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
        fi

        echo "nav2_bringup is missing, falling back to RViz with stubbed core topics." >&2
{fallback_body}
        """
    ).rstrip()


def _render_guest_demo_body() -> str:
    fallback_body = textwrap.indent(_render_guest_live_overlay_body(_fallback_live_plan()), "  ")
    return _guest_root_resolver_shell() + "\n" + textwrap.dedent(
        """\
        set -euo pipefail
        GUEST_REPO_ROOT="$(resolve_guest_repo_root)"
        if [ "$GUEST_REPO_ROOT" != "$PREFERRED_GUEST_REPO_ROOT" ]; then
          echo "Guest cannot write to $PREFERRED_GUEST_REPO_ROOT; using $GUEST_REPO_ROOT instead."
        fi
        if [ ! -f "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env" ]; then
          echo "Guest env is missing. Run ./nav2++ setup --yes first." >&2
          exit 1
        fi
        source "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env"
        export TURTLEBOT3_MODEL=waffle

        if ros2 pkg prefix nav2_bringup >/dev/null 2>&1; then
          if nav2pp_guest_has_gui; then
            exec ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False use_rviz:=True
          fi
          echo "No guest GUI display detected. Launching headless Nav2 sim." >&2
          nav2pp_headless_bootstrap_demo
          exit $?
        fi

        echo "nav2_bringup is missing in the guest, falling back to RViz with stubbed core topics." >&2
        """
    ).rstrip() + "\n" + fallback_body


def _render_live_overlay_body(repo_root: Path, plan: StartPlan) -> str:
    rviz_block = _rviz_block(
        default_config=f"{repo_root}/workspace/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz",
        opt_config="/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz",
        opt_config_fallback="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz",
    )
    return textwrap.dedent(
        f"""\
        source "$REPO_ROOT/.nav2pp/env/nav2pp.env"
        LOG_DIR="$REPO_ROOT/.nav2pp/logs/start"
        mkdir -p "$LOG_DIR"
        declare -a PIDS=()

        start_bg() {{
          local name="$1"
          shift
          "$@" >"$LOG_DIR/$name.log" 2>&1 &
          PIDS+=($!)
        }}

        cleanup() {{
          local code=$?
          if [ ${{#PIDS[@]}} -gt 0 ]; then
            kill "${{PIDS[@]}}" 2>/dev/null || true
          fi
          exit "$code"
        }}
        trap cleanup EXIT INT TERM

{textwrap.indent(_stub_block(plan.stubbed_topics), "        ")}
{textwrap.indent(rviz_block, "        ")}
        """
    ).rstrip()


def _render_guest_live_overlay_body(plan: StartPlan) -> str:
    rviz_block = _rviz_block(
        default_config="$GUEST_REPO_ROOT/workspace/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz",
        opt_config="/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz",
        opt_config_fallback="/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz",
    )
    return _guest_root_resolver_shell() + "\n" + textwrap.dedent(
        f"""\
        set -euo pipefail
        GUEST_REPO_ROOT="$(resolve_guest_repo_root)"
        if [ "$GUEST_REPO_ROOT" != "$PREFERRED_GUEST_REPO_ROOT" ]; then
          echo "Guest cannot write to $PREFERRED_GUEST_REPO_ROOT; using $GUEST_REPO_ROOT instead."
        fi
        if ! nav2pp_guest_has_gui; then
          echo "No guest GUI display detected. RViz cannot start in Lima right now." >&2
          echo "Install/configure XQuartz for X11 forwarding, or use ./nav2++ start for the headless sim path." >&2
          exit 1
        fi
        if [ ! -f "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env" ]; then
          echo "Guest env is missing. Run ./nav2++ setup --yes first." >&2
          exit 1
        fi
        source "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env"
        LOG_DIR="$GUEST_REPO_ROOT/.nav2pp/logs/start"
        mkdir -p "$LOG_DIR"
        declare -a PIDS=()

        start_bg() {{
          local name="$1"
          shift
          "$@" >"$LOG_DIR/$name.log" 2>&1 &
          PIDS+=($!)
        }}

        cleanup() {{
          local code=$?
          if [ ${{#PIDS[@]}} -gt 0 ]; then
            kill "${{PIDS[@]}}" 2>/dev/null || true
          fi
          exit "$code"
        }}
        trap cleanup EXIT INT TERM

{textwrap.indent(_stub_block(plan.stubbed_topics), "        ")}
{textwrap.indent(rviz_block, "        ")}
        """
    ).rstrip()


def _guest_root_resolver_shell() -> str:
    return textwrap.dedent(
        """\
        FALLBACK_GUEST_REPO_ROOT="$HOME/nav2pp"

        nav2pp_guest_has_gui() {
          [ -n "${DISPLAY:-}" ] || [ -n "${WAYLAND_DISPLAY:-}" ]
        }

        nav2pp_tb3_initial_pose_payload() {
          printf '%s\\n' '{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: -0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}, covariance: [0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.06853891945200942]}}'
        }

        nav2pp_seed_initial_pose() {
          local attempt=0
          local payload=""

          payload="$(nav2pp_tb3_initial_pose_payload)"
          until ros2 topic list 2>/dev/null | grep -qx '/initialpose'; do
            attempt=$((attempt + 1))
            if [ "$attempt" -ge 60 ]; then
              echo "Timed out waiting for /initialpose; skipping AMCL bootstrap." >&2
              return 1
            fi
            sleep 1
          done

          sleep 2
          echo "Publishing initial pose for headless AMCL bootstrap." >&2
          ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$payload" >/dev/null
        }

        nav2pp_headless_bootstrap_demo() {
          local launch_pid=""

          ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False &
          launch_pid=$!

          cleanup() {
            local code=$?
            if [ -n "$launch_pid" ] && kill -0 "$launch_pid" 2>/dev/null; then
              kill "$launch_pid" 2>/dev/null || true
              wait "$launch_pid" 2>/dev/null || true
            fi
            exit "$code"
          }
          trap cleanup EXIT INT TERM

          nav2pp_seed_initial_pose || true
          wait "$launch_pid"
        }

        resolve_guest_repo_root() {
          local candidate="$PREFERRED_GUEST_REPO_ROOT"
          local probe=""

          if [ -n "$candidate" ] && mkdir -p "$candidate/.nav2pp" >/dev/null 2>&1; then
            probe="$candidate/.nav2pp/.nav2pp-write-test"
            if touch "$probe" >/dev/null 2>&1; then
              rm -f "$probe"
              printf '%s\\n' "$candidate"
              return 0
            fi
          fi

          mkdir -p "$FALLBACK_GUEST_REPO_ROOT"
          printf '%s\\n' "$FALLBACK_GUEST_REPO_ROOT"
        }
        """
    ).rstrip()


def _stub_block(stubbed_topics: list[str]) -> str:
    lines: list[str] = []
    if "tf_static" in stubbed_topics:
        lines.append(
            f"start_bg tf_static ros2 topic pub -r 0.2 /tf_static tf2_msgs/msg/TFMessage '{_tf_static_payload()}'"
        )
    if "tf" in stubbed_topics:
        lines.append(
            f"start_bg tf ros2 topic pub -r 5 /tf tf2_msgs/msg/TFMessage '{_tf_payload()}'"
        )
    if "odom" in stubbed_topics:
        lines.append(
            f"start_bg odom ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry '{_odom_payload()}'"
        )
    if "map" in stubbed_topics:
        lines.append(
            f"start_bg map ros2 topic pub -r 0.5 /map nav_msgs/msg/OccupancyGrid '{_map_payload()}'"
        )
    if "scan" in stubbed_topics:
        lines.append(
            f"start_bg scan ros2 topic pub -r 5 /scan sensor_msgs/msg/LaserScan '{_scan_payload()}'"
        )
    if not lines:
        lines.append('echo "Live graph looks usable. No core topic stubs are being started."')
    return "\n".join(lines)


def _rviz_block(default_config: str, opt_config: str, opt_config_fallback: str) -> str:
    return textwrap.dedent(
        f"""\
        if ! command -v rviz2 >/dev/null 2>&1; then
          echo "rviz2 is not installed." >&2
          exit 1
        fi

        RVIZ_CONFIG="{default_config}"
        if [ ! -f "$RVIZ_CONFIG" ] && [ -f "{opt_config}" ]; then
          RVIZ_CONFIG="{opt_config}"
        fi
        if [ ! -f "$RVIZ_CONFIG" ] && [ -f "{opt_config_fallback}" ]; then
          RVIZ_CONFIG="{opt_config_fallback}"
        fi

        if [ -f "$RVIZ_CONFIG" ]; then
          rviz2 -d "$RVIZ_CONFIG"
        else
          rviz2
        fi
        """
    ).rstrip()


def _map_payload() -> str:
    zeros = ",".join(["0"] * 100)
    return (
        "{header: {frame_id: map}, info: {resolution: 0.25, width: 10, height: 10, "
        "origin: {position: {x: -1.25, y: -1.25, z: 0.0}, orientation: {w: 1.0}}}, "
        "data: [" + zeros + "]}"
    )


def _odom_payload() -> str:
    return (
        "{header: {frame_id: odom}, child_frame_id: base_link, "
        "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, "
        "twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
    )


def _scan_payload() -> str:
    ranges = ",".join(["3.0"] * 32)
    return (
        "{header: {frame_id: base_scan}, angle_min: -1.57, angle_max: 1.57, "
        "angle_increment: 0.101, time_increment: 0.0, scan_time: 0.2, "
        "range_min: 0.05, range_max: 10.0, ranges: [" + ranges + "]}"
    )


def _tf_payload() -> str:
    return (
        "{transforms: [{header: {frame_id: odom}, child_frame_id: base_link, "
        "transform: {translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {w: 1.0}}}]}"
    )


def _tf_static_payload() -> str:
    return (
        "{transforms: ["
        "{header: {frame_id: map}, child_frame_id: odom, "
        "transform: {translation: {x: 0.0, y: 0.0, z: 0.0}, rotation: {w: 1.0}}}, "
        "{header: {frame_id: base_link}, child_frame_id: base_scan, "
        "transform: {translation: {x: 0.0, y: 0.0, z: 0.2}, rotation: {w: 1.0}}}"
        "]}"
    )


def _fallback_live_plan() -> StartPlan:
    return StartPlan(
        strategy="live-rviz-overlay",
        platform_label="fallback",
        summary="Fallback live overlay",
        matched_topics={topic_type: None for topic_type in TOPIC_HINTS},
        stubbed_topics=["map", "odom", "tf", "tf_static", "scan"],
        runner_script="",
    )


def _confirm(prompt: str) -> bool:
    response = input(prompt).strip().lower()
    return response in {"", "y", "yes"}
