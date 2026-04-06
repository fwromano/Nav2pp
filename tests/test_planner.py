from __future__ import annotations

import json
import unittest
from pathlib import Path

from nav2pp_cli.bootstrap import _render_macos_docker_script, _render_macos_lima_script
from nav2pp_cli.diagnostics import HostInfo
from nav2pp_cli.planner import SetupPlan, build_setup_plan
from nav2pp_cli.topics import discover_topics, render_topic_report


class PlannerTests(unittest.TestCase):
    def test_macos_defaults_to_lima(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac14,13",
        )
        plan = build_setup_plan(host)
        self.assertEqual(plan.strategy, "macos-lima")
        self.assertEqual(plan.ros_distro, "jazzy")

    def test_macos_can_select_docker_mode(self) -> None:
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac14,13",
        )
        plan = build_setup_plan(host, mode="docker")
        self.assertEqual(plan.strategy, "macos-docker")
        self.assertEqual(plan.ros_distro, "jazzy")

    def test_ubuntu_2404_uses_jazzy(self) -> None:
        host = HostInfo(
            system="Linux",
            machine="x86_64",
            kernel_release="6.8.0",
            python_version="3.12.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="ext4",
            repo_on_removable=False,
            distro_id="ubuntu",
            distro_version="24.04",
        )
        plan = build_setup_plan(host)
        self.assertEqual(plan.strategy, "linux-native")
        self.assertEqual(plan.ros_distro, "jazzy")

    def test_small_arm_linux_uses_lite_profile(self) -> None:
        host = HostInfo(
            system="Linux",
            machine="armv7l",
            kernel_release="6.6.0",
            python_version="3.11.0",
            repo_root="/tmp/Nav2++",
            repo_filesystem="ext4",
            repo_on_removable=False,
            distro_id="ubuntu",
            distro_version="24.04",
        )
        plan = build_setup_plan(host)
        self.assertEqual(plan.strategy, "linux-native-lite")


class TopicTests(unittest.TestCase):
    def test_topic_inference_prefers_exact_matches(self) -> None:
        matches = discover_topics(topics=["scan", "/odom", "/cmd_vel", "/tf", "/tf_static"])
        self.assertEqual(matches["scan"].resolved_name, "/scan")
        self.assertEqual(matches["odom"].resolved_name, "/odom")
        self.assertEqual(matches["cmd_vel"].resolved_name, "/cmd_vel")

    def test_topic_report_json(self) -> None:
        matches = discover_topics(topics=["/scan", "/odom"])
        payload = json.loads(render_topic_report(matches, as_json=True))
        self.assertEqual(payload["scan"]["resolved_name"], "/scan")
        self.assertEqual(payload["odom"]["resolved_name"], "/odom")
        self.assertEqual(payload["cmd_vel"]["confidence"], "missing")


class MacScriptTests(unittest.TestCase):
    def test_macos_setup_script_falls_back_to_guest_home_when_mount_is_read_only(self) -> None:
        repo_root = Path.home() / "Nav2++"
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root=str(repo_root),
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
        )
        plan = SetupPlan(
            strategy="macos-lima",
            platform_label="Darwin arm64 (Mac15,14)",
            ros_distro="jazzy",
            summary="test",
        )
        script = _render_macos_lima_script(repo_root, host, plan)
        self.assertIn(f'PREFERRED_GUEST_REPO_ROOT="{repo_root}"', script)
        self.assertIn('FALLBACK_GUEST_REPO_ROOT="$HOME/nav2pp"', script)
        self.assertIn('GUEST_REPO_ROOT="$(pick_guest_repo_root)"', script)
        self.assertIn("printf '%s\\n' 'if [ -f \"$NAV2PP_WORKSPACE/install/setup.bash\" ]; then'", script)

    def test_macos_docker_setup_script_bootstraps_colima_and_build(self) -> None:
        repo_root = Path.home() / "Nav2++"
        host = HostInfo(
            system="Darwin",
            machine="arm64",
            kernel_release="24.0.0",
            python_version="3.11.0",
            repo_root=str(repo_root),
            repo_filesystem="apfs",
            repo_on_removable=False,
            hw_model="Mac15,14",
            memory_gb=32.0,
        )
        plan = SetupPlan(
            strategy="macos-docker",
            platform_label="Darwin arm64 (Mac15,14)",
            ros_distro="jazzy",
            summary="test",
        )
        script = _render_macos_docker_script(repo_root, host, plan)
        self.assertIn("brew install docker colima", script)
        self.assertIn("colima start", script)
        self.assertIn("docker build -t", script)


if __name__ == "__main__":
    unittest.main()
