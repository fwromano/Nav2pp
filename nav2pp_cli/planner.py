from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

from .diagnostics import HostInfo


@dataclass
class SetupPlan:
    strategy: str
    platform_label: str
    ros_distro: str | None
    summary: str
    rationale: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    local_steps: list[str] = field(default_factory=list)
    system_steps: list[str] = field(default_factory=list)
    next_steps: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_setup_plan(host: HostInfo, mode: str = "auto") -> SetupPlan:
    requested_mode = mode.lower()
    warnings = list(host.warnings)

    if host.system == "Darwin":
        if requested_mode == "docker":
            return SetupPlan(
                strategy="macos-docker",
                platform_label=_platform_label(host),
                ros_distro="jazzy",
                summary="Use a Docker-based Linux desktop on macOS so Nav2, Gazebo, and RViz are reachable in a browser.",
                rationale=[
                    "This keeps ROS packages out of the host while using a more standard container workflow.",
                    "A browser-served Linux desktop is a more reliable GUI path on macOS than direct Linux GUI forwarding.",
                ],
                warnings=warnings,
                local_steps=_common_local_steps(host),
                system_steps=[
                    "Install Docker CLI and Colima with Homebrew if they are not already present.",
                    "Start or reuse a Colima VM that backs the Docker engine.",
                    "Build a Nav2 desktop image with ROS 2 Jazzy, Gazebo, RViz, Nav2, and noVNC.",
                ],
                next_steps=[
                    "Run ./nav2++ start --yes to launch the browser desktop.",
                    "Open the printed localhost URL if it does not auto-open.",
                ],
            )
        if requested_mode == "native":
            warnings.append("Native macOS Nav2 is experimental and not the default path.")
            return SetupPlan(
                strategy="macos-native-experimental",
                platform_label=_platform_label(host),
                ros_distro=None,
                summary="Prepare a local tool environment only. Full Nav2 bring-up on macOS is not automated yet.",
                rationale=[
                    "ROS 2 and Nav2 are materially more reliable on Linux than on native macOS.",
                    "Apple Silicon hosts are a good fit for a lightweight Linux VM managed by Lima.",
                ],
                warnings=warnings,
                local_steps=_common_local_steps(host),
                system_steps=[],
                next_steps=[
                    "Run nav2++ setup --mode lima for the recommended path.",
                ],
            )
        return SetupPlan(
            strategy="macos-lima",
            platform_label=_platform_label(host),
            ros_distro="jazzy",
            summary="Use Lima to run an Ubuntu guest and keep Nav2 isolated from the macOS host.",
            rationale=[
                "This keeps ROS and system package churn out of the host machine.",
                "macOS hosts, especially Apple Silicon, are much easier to support through a Linux guest.",
            ],
            warnings=warnings,
            local_steps=_common_local_steps(host),
            system_steps=[
                "Install Lima with Homebrew if it is not already present.",
                "Start or reuse a lima VM named nav2pp.",
                "Inside the guest, install ROS 2 Jazzy, colcon, vcs, and rosdep.",
                "Import upstream Nav2 sources into workspace/src when they are not already present.",
            ],
            next_steps=[
                "Use ./nav2++ doctor to re-check the host after Lima is installed.",
                "Use ./nav2++ topics once your robot or simulator is publishing ROS topics.",
            ],
        )

    if host.system != "Linux":
        warnings.append(f"{host.system} is not supported yet.")
        return SetupPlan(
            strategy="unsupported",
            platform_label=_platform_label(host),
            ros_distro=None,
            summary="Host OS is unsupported for automated Nav2 bring-up.",
            warnings=warnings,
            local_steps=_common_local_steps(host),
        )

    distro_id = (host.distro_id or "").lower()
    distro_version = host.distro_version or ""
    ros_distro = _select_ros_distro(distro_id, distro_version)
    if requested_mode == "lima":
        warnings.append("Lima mode is only relevant on macOS hosts. Falling back to native Linux.")
    if requested_mode == "docker":
        warnings.append("Docker mode is only relevant on macOS hosts today. Falling back to native Linux.")

    if distro_id not in {"ubuntu", "debian"}:
        warnings.append(
            "This distro is not one of the first-class targets. The generated plan assumes apt-compatible tooling."
        )

    if host.is_wsl:
        warnings.append("WSL is workable for development, but attached robot hardware and real-time tuning are limited.")

    if host.machine in {"armv7l", "armv6l"}:
        strategy = "linux-native-lite"
        rationale = [
            "Raspberry Pi class hardware benefits from a smaller dependency footprint.",
            "This plan avoids assuming desktop-class GPU resources.",
        ]
    else:
        strategy = "linux-native"
        rationale = [
            "Linux is the straightest path for Nav2 runtime, simulation, and hardware access.",
            "The generated env stays local to the repo while ROS packages are installed through apt.",
        ]

    if ros_distro is None:
        warnings.append("ROS distro could not be inferred cleanly. Defaulting generated scripts to jazzy.")
        ros_distro = "jazzy"

    return SetupPlan(
        strategy=strategy,
        platform_label=_platform_label(host),
        ros_distro=ros_distro,
        summary="Prepare a repo-local environment and install Nav2 dependencies natively on Linux.",
        rationale=rationale,
        warnings=warnings,
        local_steps=_common_local_steps(host),
        system_steps=[
            f"Install ROS 2 {ros_distro}, colcon, vcs, and rosdep through apt.",
            "Initialize rosdep if needed and update dependency metadata.",
            "Import upstream Nav2 sources into workspace/src when they are not already present.",
            "Generate an activation script that keeps local env changes inside this repo.",
        ],
        next_steps=[
            "Source .nav2pp/env/nav2pp.env before building or launching.",
            "Run ./nav2++ topics to map your live ROS graph into common Nav2 inputs.",
        ],
    )


def _select_ros_distro(distro_id: str, distro_version: str) -> str | None:
    if distro_id == "ubuntu":
        if distro_version.startswith("24.04"):
            return "jazzy"
        if distro_version.startswith("22.04"):
            return "humble"
    if distro_id == "debian":
        return "jazzy"
    return None


def _platform_label(host: HostInfo) -> str:
    label = f"{host.system} {host.machine}"
    if host.hw_model:
        label = f"{label} ({host.hw_model})"
    return label


def _common_local_steps(host: HostInfo) -> list[str]:
    steps = [
        "Create .nav2pp/{env,generated,logs,state,venv} under the repo.",
        "Write host diagnostics and the selected plan to .nav2pp/state.",
        "Write a repo-local activation script that exports ROS and workspace paths.",
        "Ensure workspace/src exists for imported Nav2 sources and overlays.",
    ]
    if host.repo_on_removable:
        steps.append("Warn that removable media is acceptable for evaluation but not ideal for builds.")
    return steps
