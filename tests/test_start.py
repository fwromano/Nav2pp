from __future__ import annotations

import unittest
from pathlib import Path

from nav2pp_cli.diagnostics import HostInfo
from nav2pp_cli.starter import StartPlan, _render_macos_docker_start_script, _render_macos_start_script, build_start_plan
from nav2pp_cli.topics import discover_topics


class StartPlanTests(unittest.TestCase):
    def test_auto_prefers_demo_without_live_graph(self) -> None:
        host = HostInfo(
            system="Linux",
            machine="x86_64",
            kernel_release="6.8.0",
            python_version="3.12.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="ext4",
            repo_on_removable=False,
        )
        matches = discover_topics(topics=[])
        plan = build_start_plan(host, Path("/tmp/Nav2++"), matches, mode="auto")
        self.assertEqual(plan.strategy, "tb3-demo")

    def test_auto_uses_live_overlay_when_graph_exists(self) -> None:
        host = HostInfo(
            system="Linux",
            machine="x86_64",
            kernel_release="6.8.0",
            python_version="3.12.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="ext4",
            repo_on_removable=False,
        )
        matches = discover_topics(topics=["/odom", "/scan", "/tf"])
        plan = build_start_plan(host, Path("/tmp/Nav2++"), matches, mode="auto")
        self.assertEqual(plan.strategy, "live-rviz-overlay")
        self.assertIn("map", plan.stubbed_topics)
        self.assertIn("tf_static", plan.stubbed_topics)

    def test_macos_uses_guest_runner(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        matches = discover_topics(topics=[])
        plan = build_start_plan(host, Path("/tmp/Nav2++"), matches, mode="sim")
        self.assertEqual(plan.runner_script, "start-macos-lima.sh")
        self.assertEqual(plan.strategy, "tb3-demo")
        self.assertIn("headless mode otherwise", plan.summary)

    def test_macos_can_select_docker_runner_from_setup_strategy(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        matches = discover_topics(topics=[])
        plan = build_start_plan(host, Path("/tmp/Nav2++"), matches, mode="sim", setup_strategy="macos-docker")
        self.assertEqual(plan.runner_script, "start-macos-docker.sh")
        self.assertIn("browser URL", plan.summary)

    def test_macos_docker_falls_back_from_live_overlay_to_demo(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        matches = discover_topics(topics=["/odom", "/scan", "/tf"])
        plan = build_start_plan(host, Path("/tmp/Nav2++"), matches, mode="live", setup_strategy="macos-docker")
        self.assertEqual(plan.strategy, "tb3-demo")
        self.assertIn("packaged demo path only", plan.warnings[0])

    def test_macos_start_script_resolves_writable_guest_root(self) -> None:
        repo_root = Path.home() / "Nav2++"
        plan = StartPlan(
            strategy="tb3-demo",
            platform_label="Darwin arm64 (Mac15,14)",
            summary="test",
            matched_topics={},
            runner_script="start-macos-lima.sh",
        )
        script = _render_macos_start_script(repo_root, plan)
        self.assertIn(f'PREFERRED_GUEST_REPO_ROOT="{repo_root}"', script)
        self.assertIn('FALLBACK_GUEST_REPO_ROOT="$HOME/nav2pp"', script)
        self.assertIn('GUEST_REPO_ROOT="$(resolve_guest_repo_root)"', script)
        self.assertIn("nav2pp_guest_has_gui()", script)
        self.assertIn("headless:=True use_rviz:=False", script)
        self.assertIn("nav2pp_seed_initial_pose()", script)
        self.assertIn("Publishing initial pose for headless AMCL bootstrap.", script)
        self.assertIn("geometry_msgs/msg/PoseWithCovarianceStamped", script)

    def test_macos_warning_explains_host_ros2_is_guest_only(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        matches = discover_topics(topics=[])
        plan = build_start_plan(
            host,
            Path("/tmp/Nav2++"),
            matches,
            mode="auto",
            detection_warning="ros2 is not installed or not on PATH",
        )
        self.assertIn("Host ros2 is not on PATH. nav2++ will use ROS inside the Lima guest.", plan.warnings)

    def test_macos_docker_warning_explains_host_ros2_is_container_only(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        matches = discover_topics(topics=[])
        plan = build_start_plan(
            host,
            Path("/tmp/Nav2++"),
            matches,
            mode="auto",
            detection_warning="ros2 is not installed or not on PATH",
            setup_strategy="macos-docker",
        )
        self.assertIn("Host ros2 is not on PATH. nav2++ will use ROS inside the Docker runtime.", plan.warnings)

    def test_macos_docker_start_script_rebuilds_image(self) -> None:
        repo_root = Path.home() / "Nav2++"
        plan = StartPlan(
            strategy="tb3-demo",
            platform_label="Darwin arm64 (Mac15,14)",
            summary="test",
            matched_topics={},
            runner_script="start-macos-docker.sh",
        )
        script = _render_macos_docker_start_script(repo_root, plan)
        self.assertIn('docker build -t "${IMAGE_TAG}" -f "${REPO_ROOT}/docker/nav2pp-desktop/Dockerfile" "${REPO_ROOT}"', script)
        self.assertNotIn('docker image inspect "${IMAGE_TAG}"', script)
        self.assertIn('BROWSER_URL="http://127.0.0.1:6080/vnc.html?autoconnect=1&resize=scale"', script)

    def test_browser_launch_uses_nav2_boolean_helpers(self) -> None:
        content = Path("docker/nav2pp-desktop/launch/nav2pp_navigation_launch.py").read_text()
        self.assertIn("LaunchConfigAsBool", content)
        self.assertIn('default_value="False"', content)
        self.assertIn('default_value="True"', content)

    def test_browser_launch_cleanup_handles_missing_temp_world(self) -> None:
        content = Path("docker/nav2pp-desktop/launch/nav2pp_tb3_browser_launch.py").read_text()
        self.assertIn("_remove_file_if_present", content)
        self.assertIn("os.path.exists(path)", content)


if __name__ == "__main__":
    unittest.main()
