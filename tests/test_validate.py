from __future__ import annotations

import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from nav2pp_cli.validator import (
    ResourceEndpoint,
    load_topic_snapshot,
    render_validation_report,
    validate_profile,
)


class TopicSnapshotTests(unittest.TestCase):
    def test_load_topic_snapshot_supports_colon_and_bracket_entries(self) -> None:
        endpoints = load_topic_snapshot(
            topics=[
                "/map [nav_msgs/msg/OccupancyGrid]",
                "odom:nav_msgs/msg/Odometry",
                "/scan:sensor_msgs/msg/LaserScan",
            ]
        )
        self.assertEqual(endpoints["/map"].msg_type, "nav_msgs/msg/OccupancyGrid")
        self.assertEqual(endpoints["/odom"].msg_type, "nav_msgs/msg/Odometry")
        self.assertEqual(endpoints["/scan"].msg_type, "sensor_msgs/msg/LaserScan")


class ValidationTests(unittest.TestCase):
    def test_vehicle_profile_passes_from_snapshot(self) -> None:
        report = validate_profile(
            "vehicle",
            topics=[
                "/map:nav_msgs/msg/OccupancyGrid",
                "/odometry/filtered:nav_msgs/msg/Odometry",
                "/tf:tf2_msgs/msg/TFMessage",
                "/tf_static:tf2_msgs/msg/TFMessage",
                "/nav/cmd_vel:geometry_msgs/msg/Twist",
                "/lidar/scan:sensor_msgs/msg/LaserScan",
                "/imu/data:sensor_msgs/msg/Imu",
            ],
        )
        self.assertTrue(report.passed)
        self.assertEqual(report.profile, "vehicle")
        self.assertIn(
            "Live frame, TF, action, and host probes were skipped because a topic snapshot was supplied.",
            report.warnings,
        )

    def test_vehicle_profile_fails_with_missing_core_inputs(self) -> None:
        report = validate_profile(
            "vehicle",
            topics=[
                "/map:std_msgs/msg/String",
                "/odom:nav_msgs/msg/Odometry",
                "/tf:tf2_msgs/msg/TFMessage",
            ],
        )
        self.assertFalse(report.passed)
        failures = {
            check.name: check.detail
            for check in report.checks
            if check.required and check.status == "fail"
        }
        self.assertIn("map", failures)
        self.assertIn("tf_static", failures)
        self.assertIn("cmd_vel", failures)
        self.assertIn("sensor_input", failures)

    @patch("nav2pp_cli.validator._load_action_snapshot")
    @patch("nav2pp_cli.validator._tf_echo_available")
    @patch("nav2pp_cli.validator._read_topic_field_once")
    @patch("nav2pp_cli.validator._run_ros2_topic_list_t")
    def test_nav2_profile_live_probe_checks_tf_frames_and_action(
        self,
        mock_topic_list: unittest.mock.Mock,
        mock_read_field: unittest.mock.Mock,
        mock_tf_echo: unittest.mock.Mock,
        mock_actions: unittest.mock.Mock,
    ) -> None:
        mock_topic_list.return_value = {
            "/map": ResourceEndpoint("/map", "nav_msgs/msg/OccupancyGrid"),
            "/odom": ResourceEndpoint("/odom", "nav_msgs/msg/Odometry"),
            "/tf": ResourceEndpoint("/tf", "tf2_msgs/msg/TFMessage"),
            "/tf_static": ResourceEndpoint("/tf_static", "tf2_msgs/msg/TFMessage"),
            "/cmd_vel": ResourceEndpoint("/cmd_vel", "geometry_msgs/msg/Twist"),
            "/scan": ResourceEndpoint("/scan", "sensor_msgs/msg/LaserScan"),
        }
        mock_read_field.side_effect = lambda topic, field: {
            ("/map", "header.frame_id"): "map",
            ("/odom", "header.frame_id"): "odom",
            ("/odom", "child_frame_id"): "base_link",
            ("/scan", "header.frame_id"): "base_scan",
        }.get((topic, field))
        mock_tf_echo.side_effect = lambda target, source: {
            ("map", "odom"): True,
            ("odom", "base_link"): True,
            ("odom", "base_footprint"): False,
        }.get((target, source), False)
        mock_actions.return_value = {
            "/navigate_to_pose": ResourceEndpoint(
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
            )
        }

        report = validate_profile("nav2")

        self.assertTrue(report.passed)
        action_check = next(check for check in report.checks if check.name == "action_navigate_to_pose")
        self.assertEqual(action_check.status, "pass")
        self.assertEqual(report.warnings, [])

    def test_render_validation_report_mentions_result(self) -> None:
        report = validate_profile(
            "vehicle",
            topics=[
                "/map:nav_msgs/msg/OccupancyGrid",
                "/odom:nav_msgs/msg/Odometry",
                "/tf:tf2_msgs/msg/TFMessage",
                "/tf_static:tf2_msgs/msg/TFMessage",
                "/cmd_vel:geometry_msgs/msg/Twist",
                "/scan:sensor_msgs/msg/LaserScan",
            ],
        )
        rendered = render_validation_report(report, as_json=False)
        self.assertIn("Result: pass", rendered)
        self.assertIn("sensor_input", rendered)

    def test_custom_profile_file_can_require_exact_dbw_topic(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            profile_path = repo_root / "jeep.json"
            profile_path.write_text(
                json.dumps(
                    {
                        "name": "jeep",
                        "extends": "vehicle",
                        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel", "dbw_cmd"],
                        "resource_hints": {
                            "dbw_cmd": ["/vehicle/dbw/command"],
                        },
                        "expected_topic_types": {
                            "dbw_cmd": ["ackermann_msgs/msg/AckermannDriveStamped"],
                        },
                    }
                )
            )

            report = validate_profile(
                "jeep",
                profile_file=profile_path,
                repo_root=repo_root,
                topics=[
                    "/map:nav_msgs/msg/OccupancyGrid",
                    "/odom:nav_msgs/msg/Odometry",
                    "/tf:tf2_msgs/msg/TFMessage",
                    "/tf_static:tf2_msgs/msg/TFMessage",
                    "/cmd_vel:geometry_msgs/msg/Twist",
                    "/scan:sensor_msgs/msg/LaserScan",
                    "/vehicle/dbw/command:ackermann_msgs/msg/AckermannDriveStamped",
                ],
            )

            self.assertTrue(report.passed)
            dbw_check = next(check for check in report.checks if check.name == "dbw_cmd")
            self.assertEqual(dbw_check.status, "pass")

    def test_named_profile_is_loaded_from_repo_profiles_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            profiles_dir = repo_root / "profiles"
            profiles_dir.mkdir(parents=True)
            (profiles_dir / "jeep.json").write_text(
                json.dumps(
                    {
                        "name": "jeep",
                        "extends": "vehicle",
                        "required_roles": ["map", "odom", "tf", "tf_static", "cmd_vel", "vehicle_status"],
                        "resource_hints": {
                            "vehicle_status": ["/vehicle/status"],
                        },
                        "expected_topic_types": {
                            "vehicle_status": ["std_msgs/msg/String"],
                        },
                    }
                )
            )

            report = validate_profile(
                "jeep",
                repo_root=repo_root,
                topics=[
                    "/map:nav_msgs/msg/OccupancyGrid",
                    "/odom:nav_msgs/msg/Odometry",
                    "/tf:tf2_msgs/msg/TFMessage",
                    "/tf_static:tf2_msgs/msg/TFMessage",
                    "/cmd_vel:geometry_msgs/msg/Twist",
                    "/scan:sensor_msgs/msg/LaserScan",
                    "/vehicle/status:std_msgs/msg/String",
                ],
            )

            self.assertTrue(report.passed)
            self.assertEqual(report.profile, "jeep")

    @patch("nav2pp_cli.validator._run_host_command")
    @patch("nav2pp_cli.validator._load_action_snapshot")
    @patch("nav2pp_cli.validator._tf_echo_available")
    @patch("nav2pp_cli.validator._read_topic_field_once")
    @patch("nav2pp_cli.validator._run_ros2_topic_list_t")
    def test_custom_profile_host_check_can_validate_nvidia(
        self,
        mock_topic_list: unittest.mock.Mock,
        mock_read_field: unittest.mock.Mock,
        mock_tf_echo: unittest.mock.Mock,
        mock_actions: unittest.mock.Mock,
        mock_host_command: unittest.mock.Mock,
    ) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            profiles_dir = repo_root / "profiles"
            profiles_dir.mkdir(parents=True)
            (profiles_dir / "jeep.json").write_text(
                json.dumps(
                    {
                        "name": "jeep",
                        "extends": "nav2",
                        "host_checks": [
                            {
                                "name": "nvidia_gpu",
                                "command": ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                                "nonempty_stdout": True,
                                "required": True,
                            }
                        ],
                    }
                )
            )

            mock_topic_list.return_value = {
                "/map": ResourceEndpoint("/map", "nav_msgs/msg/OccupancyGrid"),
                "/odom": ResourceEndpoint("/odom", "nav_msgs/msg/Odometry"),
                "/tf": ResourceEndpoint("/tf", "tf2_msgs/msg/TFMessage"),
                "/tf_static": ResourceEndpoint("/tf_static", "tf2_msgs/msg/TFMessage"),
                "/cmd_vel": ResourceEndpoint("/cmd_vel", "geometry_msgs/msg/Twist"),
                "/scan": ResourceEndpoint("/scan", "sensor_msgs/msg/LaserScan"),
            }
            mock_read_field.side_effect = lambda topic, field: {
                ("/map", "header.frame_id"): "map",
                ("/odom", "header.frame_id"): "odom",
                ("/odom", "child_frame_id"): "base_link",
                ("/scan", "header.frame_id"): "base_scan",
            }.get((topic, field))
            mock_tf_echo.side_effect = lambda target, source: {
                ("map", "odom"): True,
                ("odom", "base_link"): True,
                ("odom", "base_footprint"): False,
            }.get((target, source), False)
            mock_actions.return_value = {
                "/navigate_to_pose": ResourceEndpoint(
                    "/navigate_to_pose",
                    "nav2_msgs/action/NavigateToPose",
                )
            }
            mock_host_command.return_value = subprocess.CompletedProcess(
                ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                0,
                stdout="NVIDIA RTX 6000 Ada\n",
                stderr="",
            )

            report = validate_profile("jeep", repo_root=repo_root)

            self.assertTrue(report.passed)
            nvidia_check = next(check for check in report.checks if check.name == "nvidia_gpu")
            self.assertEqual(nvidia_check.status, "pass")
            self.assertIn("NVIDIA RTX 6000 Ada", nvidia_check.detail)


if __name__ == "__main__":
    unittest.main()
