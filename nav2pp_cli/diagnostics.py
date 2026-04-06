from __future__ import annotations

import json
import platform
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class HostInfo:
    system: str
    machine: str
    kernel_release: str
    python_version: str
    repo_root: str
    repo_filesystem: str | None
    repo_on_removable: bool
    cpu_model: str | None = None
    hw_model: str | None = None
    distro_id: str | None = None
    distro_version: str | None = None
    distro_codename: str | None = None
    memory_gb: float | None = None
    disk_free_gb: float | None = None
    package_managers: dict[str, str] = field(default_factory=dict)
    tools: dict[str, str] = field(default_factory=dict)
    ros_tools: dict[str, str] = field(default_factory=dict)
    gpus: list[str] = field(default_factory=list)
    is_rosetta: bool = False
    is_wsl: bool = False
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)


def collect_host_info(repo_root: Path) -> HostInfo:
    repo_root = repo_root.resolve()
    system = platform.system()
    machine = platform.machine().lower()
    info = HostInfo(
        system=system,
        machine=machine,
        kernel_release=platform.release(),
        python_version=platform.python_version(),
        repo_root=str(repo_root),
        repo_filesystem=_detect_repo_filesystem(repo_root),
        repo_on_removable=_repo_on_removable(repo_root, system),
    )
    info.package_managers = _detect_commands(["apt-get", "brew", "dnf", "yum", "pacman"])
    info.tools = _detect_commands(["git", "curl", "docker", "colima", "limactl", "nvidia-smi", "lspci"])
    info.ros_tools = _detect_commands(["ros2", "colcon", "vcs", "rosdep"])
    info.is_rosetta = _detect_rosetta(system)
    info.is_wsl = _detect_wsl(system, info.kernel_release)
    info.cpu_model = _detect_cpu_model(system)
    info.hw_model = _detect_hw_model(system)
    info.memory_gb = _detect_memory_gb(system)
    info.disk_free_gb = _detect_disk_free_gb(repo_root)
    info.gpus = _detect_gpu_names(system)

    if system == "Linux":
        distro = _detect_linux_distro()
        info.distro_id = distro.get("id")
        info.distro_version = distro.get("version_id")
        info.distro_codename = distro.get("version_codename")
    elif system == "Darwin" and info.repo_on_removable:
        info.warnings.append(
            "Repo is on a removable volume. That works, but Lima host mounts and build speed "
            "are more predictable when the repo lives under your home directory."
        )

    if info.is_rosetta:
        info.warnings.append("This shell is running under Rosetta. Native arm64 tools will be more predictable.")
    if (info.disk_free_gb or 0) < 20:
        info.warnings.append("Less than 20 GB free in the repo filesystem. Nav2 source builds will be cramped.")

    return info


def _detect_commands(commands: list[str]) -> dict[str, str]:
    found: dict[str, str] = {}
    for command in commands:
        path = shutil.which(command)
        if path:
            found[command] = path
    return found


def _run_capture(command: list[str], timeout: float = 3.0) -> str | None:
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout,
        )
    except (FileNotFoundError, subprocess.SubprocessError, OSError):
        return None

    if completed.returncode != 0:
        return None

    output = completed.stdout.strip()
    if output.startswith("sysctl:") or "Operation not permitted" in output:
        return None
    return output or None


def _detect_cpu_model(system: str) -> str | None:
    if system == "Darwin":
        return _run_capture(["sysctl", "-n", "machdep.cpu.brand_string"]) or _detect_macos_hardware_field("Chip")
    if system == "Linux":
        cpuinfo = Path("/proc/cpuinfo")
        if cpuinfo.exists():
            for line in cpuinfo.read_text().splitlines():
                if line.lower().startswith("model name"):
                    return line.split(":", 1)[1].strip()
    return platform.processor() or None


def _detect_hw_model(system: str) -> str | None:
    if system == "Darwin":
        return _run_capture(["sysctl", "-n", "hw.model"]) or _detect_macos_hardware_field("Model Identifier")
    if system == "Linux":
        model_path = Path("/sys/devices/virtual/dmi/id/product_name")
        if model_path.exists():
            return model_path.read_text().strip()
    return None


def _detect_memory_gb(system: str) -> float | None:
    if system == "Darwin":
        raw = _run_capture(["sysctl", "-n", "hw.memsize"])
        if raw and raw.isdigit():
            return round(int(raw) / (1024**3), 1)
        raw = _detect_macos_hardware_field("Memory")
        if raw:
            value, unit = raw.split(maxsplit=1)
            if unit.upper().startswith("GB"):
                return float(value)
    if system == "Linux":
        meminfo = Path("/proc/meminfo")
        if meminfo.exists():
            for line in meminfo.read_text().splitlines():
                if line.startswith("MemTotal:"):
                    parts = line.split()
                    if len(parts) >= 2 and parts[1].isdigit():
                        return round((int(parts[1]) * 1024) / (1024**3), 1)
    return None


def _detect_disk_free_gb(repo_root: Path) -> float | None:
    try:
        usage = shutil.disk_usage(repo_root)
    except OSError:
        return None
    return round(usage.free / (1024**3), 1)


def _detect_linux_distro() -> dict[str, str]:
    distro: dict[str, str] = {}
    os_release = Path("/etc/os-release")
    if not os_release.exists():
        return distro
    for line in os_release.read_text().splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        distro[key.lower()] = value.strip().strip('"')
    return distro


def _detect_rosetta(system: str) -> bool:
    if system != "Darwin":
        return False
    output = _run_capture(["sysctl", "-n", "sysctl.proc_translated"])
    return output == "1"


def _detect_wsl(system: str, kernel_release: str) -> bool:
    if system != "Linux":
        return False
    return "microsoft" in kernel_release.lower()


def _detect_gpu_names(system: str) -> list[str]:
    gpus: list[str] = []
    nvidia_output = _run_capture(
        ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
        timeout=5.0,
    )
    if nvidia_output:
        gpus.extend([line.strip() for line in nvidia_output.splitlines() if line.strip()])
    if system == "Darwin":
        profiler = _run_capture(["system_profiler", "SPDisplaysDataType"], timeout=8.0)
        if profiler:
            for line in profiler.splitlines():
                stripped = line.strip()
                if stripped.startswith("Chipset Model:"):
                    gpus.append(stripped.split(":", 1)[1].strip())
    elif system == "Linux":
        lspci = _run_capture(["lspci"], timeout=5.0)
        if lspci:
            for line in lspci.splitlines():
                lower = line.lower()
                if "vga compatible controller" in lower or "3d controller" in lower:
                    gpus.append(line.split(":", 2)[-1].strip())

    deduped: list[str] = []
    for gpu in gpus:
        if gpu not in deduped:
            deduped.append(gpu)
    return deduped


def _detect_macos_hardware_field(field: str) -> str | None:
    profiler = _run_capture(["system_profiler", "SPHardwareDataType"], timeout=8.0)
    if not profiler:
        return None
    prefix = f"{field}:"
    for line in profiler.splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            return stripped.split(":", 1)[1].strip()
    return None


def _detect_repo_filesystem(repo_root: Path) -> str | None:
    if sys.platform == "darwin":
        result = _run_capture(["stat", "-f", "%T", str(repo_root)])
        if result in {"", "/"}:
            return None
        return result or None
    result = _run_capture(["stat", "-f", "-c", "%T", str(repo_root)])
    return result or None


def _repo_on_removable(repo_root: Path, system: str) -> bool:
    parts = repo_root.parts
    if system == "Darwin":
        return len(parts) > 1 and parts[1] == "Volumes"
    if system == "Linux":
        return any(part in {"media", "mnt", "run"} for part in parts[:3])
    return False
