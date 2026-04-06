from __future__ import annotations

import json
import subprocess
import sys
import textwrap
from pathlib import Path

from .diagnostics import HostInfo, collect_host_info
from .planner import SetupPlan, build_setup_plan


def run_setup(
    repo_root: Path,
    mode: str,
    report_only: bool,
    yes: bool,
) -> tuple[HostInfo, SetupPlan]:
    repo_root = repo_root.resolve()
    host = collect_host_info(repo_root)
    plan = build_setup_plan(host, mode=mode)

    _ensure_repo_layout(repo_root)
    _ensure_local_venv(repo_root)
    _write_state_files(repo_root, host, plan)
    _write_generated_scripts(repo_root, host, plan)

    if report_only:
        return host, plan

    runner = _select_runner(repo_root, plan)
    if runner is None:
        return host, plan

    if not yes and sys.stdin.isatty():
        if not _confirm(f"Run generated setup script now ({runner.name})? [Y/n] "):
            return host, plan
    elif not yes and not sys.stdin.isatty():
        print(
            "Skipping system package installation because this session is non-interactive. "
            "Re-run with --yes to execute the generated script.",
            file=sys.stderr,
        )
        return host, plan

    subprocess.run([str(runner)], check=True)
    return host, plan


def _ensure_repo_layout(repo_root: Path) -> None:
    paths = [
        repo_root / ".nav2pp",
        repo_root / ".nav2pp" / "env",
        repo_root / ".nav2pp" / "generated",
        repo_root / ".nav2pp" / "logs",
        repo_root / ".nav2pp" / "state",
        repo_root / ".nav2pp" / "venv",
        repo_root / ".nav2pp" / "state" / "colcon",
        repo_root / "workspace" / "src",
        repo_root / "manifests",
    ]
    for path in paths:
        path.mkdir(parents=True, exist_ok=True)
    _write_activation_script(repo_root)
    _write_nav2_manifest(repo_root)


def _ensure_local_venv(repo_root: Path) -> None:
    venv_dir = repo_root / ".nav2pp" / "venv"
    marker = venv_dir / "pyvenv.cfg"
    if marker.exists():
        return
    subprocess.run([sys.executable, "-m", "venv", str(venv_dir)], check=True)


def _write_state_files(repo_root: Path, host: HostInfo, plan: SetupPlan) -> None:
    state_dir = repo_root / ".nav2pp" / "state"
    (state_dir / "host.json").write_text(host.to_json() + "\n")
    (state_dir / "plan.json").write_text(json.dumps(plan.to_dict(), indent=2, sort_keys=True) + "\n")


def _write_generated_scripts(repo_root: Path, host: HostInfo, plan: SetupPlan) -> None:
    generated_dir = repo_root / ".nav2pp" / "generated"
    if plan.strategy in {"linux-native", "linux-native-lite"}:
        script = _render_linux_native_script(repo_root, plan)
        path = generated_dir / "setup-native-linux.sh"
    elif plan.strategy == "macos-docker":
        script = _render_macos_docker_script(repo_root, host, plan)
        path = generated_dir / "setup-macos-docker.sh"
    elif plan.strategy == "macos-lima":
        script = _render_macos_lima_script(repo_root, host, plan)
        path = generated_dir / "setup-macos-lima.sh"
    else:
        script = "#!/usr/bin/env bash\nset -euo pipefail\nprintf 'No automated system installer for this host yet.\\n'\n"
        path = generated_dir / "setup-unsupported.sh"
    path.write_text(script)
    path.chmod(0o755)


def _write_activation_script(repo_root: Path) -> None:
    env_path = repo_root / ".nav2pp" / "env" / "nav2pp.env"
    contents = textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        # shellcheck shell=bash
        export NAV2PP_ROOT="{repo_root}"
        export NAV2PP_STATE_DIR="{repo_root / '.nav2pp' / 'state'}"
        export NAV2PP_WORKSPACE="{repo_root / 'workspace'}"
        export NAV2PP_VENV="{repo_root / '.nav2pp' / 'venv'}"

        nav2pp_safe_source() {{
          local target="$1"
          local had_u=0
          case $- in
            *u*) had_u=1 ;;
          esac
          set +u
          # shellcheck disable=SC1090
          source "$target"
          if [ "$had_u" -eq 1 ]; then
            set -u
          fi
        }}

        if [ -f "/opt/ros/jazzy/setup.bash" ]; then
          nav2pp_safe_source /opt/ros/jazzy/setup.bash
        elif [ -f "/opt/ros/humble/setup.bash" ]; then
          nav2pp_safe_source /opt/ros/humble/setup.bash
        fi

        if [ -f "${{NAV2PP_WORKSPACE}}/install/setup.bash" ]; then
          nav2pp_safe_source "${{NAV2PP_WORKSPACE}}/install/setup.bash"
        fi

        export PATH="${{NAV2PP_VENV}}/bin:${{PATH}}"
        export COLCON_HOME="{repo_root / '.nav2pp' / 'state' / 'colcon'}"
        export ROS_LOG_DIR="{repo_root / '.nav2pp' / 'logs'}"
        """
    )
    env_path.write_text(contents)
    env_path.chmod(0o755)


def _write_nav2_manifest(repo_root: Path) -> None:
    manifest_path = repo_root / "manifests" / "nav2.repos"
    if manifest_path.exists():
        return
    manifest_path.write_text(
        textwrap.dedent(
            """\
            repositories:
              navigation2:
                type: git
                url: https://github.com/ros-navigation/navigation2.git
                version: main
            """
        )
    )


def _render_linux_native_script(repo_root: Path, plan: SetupPlan) -> str:
    ros_distro = plan.ros_distro or "jazzy"
    return textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        set -euo pipefail

        REPO_ROOT="{repo_root}"
        ROS_DISTRO="{ros_distro}"
        ROS_APT_SOURCE="/etc/apt/sources.list.d/ros2.list"
        KEYRING="/usr/share/keyrings/ros-archive-keyring.gpg"

        if ! command -v apt-get >/dev/null 2>&1; then
          echo "apt-get is required for the native Linux installer." >&2
          exit 1
        fi

        sudo apt-get update
        sudo apt-get install -y curl gnupg lsb-release software-properties-common ca-certificates

        if [ ! -f "${{KEYRING}}" ]; then
          sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "${{KEYRING}}"
        fi

        if [ ! -f "${{ROS_APT_SOURCE}}" ]; then
          echo "deb [arch=$(dpkg --print-architecture) signed-by=${{KEYRING}}] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${{UBUNTU_CODENAME:-$VERSION_CODENAME}}) main" | sudo tee "${{ROS_APT_SOURCE}}" >/dev/null
        fi

        sudo apt-get update
        sudo apt-get install -y \
          build-essential \
          cmake \
          git \
          python3-argcomplete \
          python3-colcon-common-extensions \
          python3-pip \
          python3-rosdep \
          python3-vcstool \
          ros-${{ROS_DISTRO}}-desktop \
          ros-${{ROS_DISTRO}}-navigation2 \
          ros-${{ROS_DISTRO}}-nav2-bringup

        if apt-cache pkgnames | grep -q "^ros-${{ROS_DISTRO}}-nav2-minimal-tb"; then
          sudo apt-get install -y ros-${{ROS_DISTRO}}-nav2-minimal-tb*
        elif apt-cache pkgnames | grep -q "^ros-${{ROS_DISTRO}}-turtlebot3-gazebo$"; then
          sudo apt-get install -y ros-${{ROS_DISTRO}}-turtlebot3-gazebo
        fi

        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
          sudo rosdep init
        fi
        rosdep update

        mkdir -p "${{REPO_ROOT}}/workspace/src"
        if [ ! -d "${{REPO_ROOT}}/workspace/src/navigation2" ]; then
          vcs import "${{REPO_ROOT}}/workspace/src" < "${{REPO_ROOT}}/manifests/nav2.repos"
        fi

        echo
        echo "Native Linux bootstrap complete."
        echo "Source ${{REPO_ROOT}}/.nav2pp/env/nav2pp.env before building or launching."
        """
    )


def _render_macos_lima_script(repo_root: Path, host: HostInfo, plan: SetupPlan) -> str:
    ros_distro = plan.ros_distro or "jazzy"
    repo_mountable = str(repo_root).startswith(str(Path.home()))
    guest_repo_root = str(repo_root) if repo_mountable else "$HOME/nav2pp"
    guest_manifest_bootstrap = textwrap.dedent(
        """\
        FALLBACK_GUEST_REPO_ROOT="$HOME/nav2pp"

        pick_guest_repo_root() {
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

        GUEST_REPO_ROOT="$(pick_guest_repo_root)"
        if [ "$GUEST_REPO_ROOT" != "$PREFERRED_GUEST_REPO_ROOT" ]; then
          echo "Guest cannot write to $PREFERRED_GUEST_REPO_ROOT; using $GUEST_REPO_ROOT instead."
        fi
        echo "Guest root: $GUEST_REPO_ROOT"

        mkdir -p "$GUEST_REPO_ROOT/manifests"
        if [ ! -f "$GUEST_REPO_ROOT/manifests/nav2.repos" ]; then
          cat > "$GUEST_REPO_ROOT/manifests/nav2.repos" <<'NAV2PP_REPOS_EOF'
        repositories:
          navigation2:
            type: git
            url: https://github.com/ros-navigation/navigation2.git
            version: main
        NAV2PP_REPOS_EOF
        fi
        mkdir -p "$GUEST_REPO_ROOT/.nav2pp/env" "$GUEST_REPO_ROOT/.nav2pp/logs" "$GUEST_REPO_ROOT/.nav2pp/state/colcon"
        {
          printf '%s\\n' '#!/usr/bin/env bash'
          printf '%s\\n' "export NAV2PP_ROOT=\\"$GUEST_REPO_ROOT\\""
          printf '%s\\n' "export NAV2PP_STATE_DIR=\\"$GUEST_REPO_ROOT/.nav2pp/state\\""
          printf '%s\\n' "export NAV2PP_WORKSPACE=\\"$GUEST_REPO_ROOT/workspace\\""
          printf '%s\\n' ''
          printf '%s\\n' 'nav2pp_safe_source() {'
          printf '%s\\n' '  local target="$1"'
          printf '%s\\n' '  local had_u=0'
          printf '%s\\n' '  case $- in'
          printf '%s\\n' '    *u*) had_u=1 ;;'
          printf '%s\\n' '  esac'
          printf '%s\\n' '  set +u'
          printf '%s\\n' '  source "$target"'
          printf '%s\\n' '  if [ "$had_u" -eq 1 ]; then'
          printf '%s\\n' '    set -u'
          printf '%s\\n' '  fi'
          printf '%s\\n' '}'
          printf '%s\\n' ''
          printf '%s\\n' "if [ -f \\"/opt/ros/$ROS_DISTRO/setup.bash\\" ]; then"
          printf '%s\\n' "  nav2pp_safe_source \\"/opt/ros/$ROS_DISTRO/setup.bash\\""
          printf '%s\\n' 'fi'
          printf '%s\\n' 'if [ -f "$NAV2PP_WORKSPACE/install/setup.bash" ]; then'
          printf '%s\\n' '  nav2pp_safe_source "$NAV2PP_WORKSPACE/install/setup.bash"'
          printf '%s\\n' 'fi'
          printf '%s\\n' "export COLCON_HOME=\\"$GUEST_REPO_ROOT/.nav2pp/state/colcon\\""
          printf '%s\\n' "export ROS_LOG_DIR=\\"$GUEST_REPO_ROOT/.nav2pp/logs\\""
        } > "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env"
        chmod +x "$GUEST_REPO_ROOT/.nav2pp/env/nav2pp.env"
        """
    ).rstrip()
    repo_note = ""
    if not repo_mountable:
        repo_note = 'echo "Repo is outside $HOME; guest setup will use $HOME/nav2pp inside the VM."'
    return textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        set -euo pipefail

        ROS_DISTRO="{ros_distro}"
        HOST_REPO_ROOT="{repo_root}"
        PREFERRED_GUEST_REPO_ROOT="{guest_repo_root}"
        {repo_note}

        if ! command -v brew >/dev/null 2>&1; then
          echo "Homebrew is required on macOS for the Lima-based installer." >&2
          exit 1
        fi

        brew install lima

        if ! limactl list | awk '{{print $1}}' | grep -qx nav2pp; then
          limactl start --name nav2pp template://ubuntu-24.04
        else
          limactl start nav2pp
        fi

        limactl shell nav2pp -- env ROS_DISTRO="${{ROS_DISTRO}}" PREFERRED_GUEST_REPO_ROOT="${{PREFERRED_GUEST_REPO_ROOT}}" bash -s <<'EOF'
        set -euo pipefail
        export DEBIAN_FRONTEND=noninteractive
        sudo apt-get update
        sudo apt-get install -y curl gnupg lsb-release software-properties-common ca-certificates
        sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
        sudo apt-get update
        sudo apt-get install -y build-essential cmake git python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
        if apt-cache pkgnames | grep -q "^ros-$ROS_DISTRO-nav2-minimal-tb"; then
          sudo apt-get install -y ros-$ROS_DISTRO-nav2-minimal-tb*
        elif apt-cache pkgnames | grep -q "^ros-$ROS_DISTRO-turtlebot3-gazebo$"; then
          sudo apt-get install -y ros-$ROS_DISTRO-turtlebot3-gazebo
        fi
        if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
          sudo rosdep init
        fi
        rosdep update
{textwrap.indent(guest_manifest_bootstrap, "        ")}
        mkdir -p "$GUEST_REPO_ROOT/workspace/src"
        if [ ! -d "$GUEST_REPO_ROOT/workspace/src/navigation2" ]; then
          vcs import "$GUEST_REPO_ROOT/workspace/src" < "$GUEST_REPO_ROOT/manifests/nav2.repos"
        fi
        EOF

        echo "Lima bootstrap complete."
        echo "Use limactl shell nav2pp to enter the guest."
        echo "If the mounted repo was not writable in the guest, nav2++ used \$HOME/nav2pp instead."
        """
    )


def _render_macos_docker_script(repo_root: Path, host: HostInfo, plan: SetupPlan) -> str:
    ros_distro = plan.ros_distro or "jazzy"
    cpu_count = max(4, min(8, (host.memory_gb and int(host.memory_gb // 8) * 2) or 6))
    memory_gb = max(8, min(16, int((host.memory_gb or 16) // 2) or 12))
    return textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        set -euo pipefail

        REPO_ROOT="{repo_root}"
        ROS_DISTRO="{ros_distro}"
        IMAGE_TAG="nav2pp-jazzy-desktop:local"
        COLIMA_CPUS="{cpu_count}"
        COLIMA_MEMORY_GB="{memory_gb}"
        COLIMA_DISK_GB="80"

        if ! command -v brew >/dev/null 2>&1; then
          echo "Homebrew is required on macOS for the Docker-based installer." >&2
          exit 1
        fi

        brew install docker colima

        if ! docker info >/dev/null 2>&1; then
          colima start --cpu "${{COLIMA_CPUS}}" --memory "${{COLIMA_MEMORY_GB}}" --disk "${{COLIMA_DISK_GB}}"
        fi

        if ! docker info >/dev/null 2>&1; then
          echo "Docker engine is still unavailable after attempting to start Colima." >&2
          exit 1
        fi

        docker build -t "${{IMAGE_TAG}}" -f "${{REPO_ROOT}}/docker/nav2pp-desktop/Dockerfile" "${{REPO_ROOT}}"

        echo "Docker bootstrap complete."
        echo "Run ./nav2++ start --yes to launch the browser desktop at http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale"
        """
    )


def _select_runner(repo_root: Path, plan: SetupPlan) -> Path | None:
    generated_dir = repo_root / ".nav2pp" / "generated"
    mapping = {
        "linux-native": generated_dir / "setup-native-linux.sh",
        "linux-native-lite": generated_dir / "setup-native-linux.sh",
        "macos-docker": generated_dir / "setup-macos-docker.sh",
        "macos-lima": generated_dir / "setup-macos-lima.sh",
        "macos-native-experimental": None,
        "unsupported": None,
    }
    return mapping.get(plan.strategy)


def _confirm(prompt: str) -> bool:
    response = input(prompt).strip().lower()
    return response in {"", "y", "yes"}
