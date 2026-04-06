from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from nav2pp_cli.profile_scaffold import scaffold_profile


class ProfileScaffoldTests(unittest.TestCase):
    def test_scaffold_writes_profile_with_detected_topics(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            output_path, notes = scaffold_profile(
                "jeep",
                repo_root=repo_root,
                topics=[
                    "/map:nav_msgs/msg/OccupancyGrid",
                    "/odometry/filtered:nav_msgs/msg/Odometry",
                    "/tf:tf2_msgs/msg/TFMessage",
                    "/tf_static:tf2_msgs/msg/TFMessage",
                    "/cmd_vel:geometry_msgs/msg/Twist",
                    "/front_lidar/scan:sensor_msgs/msg/LaserScan",
                    "/vehicle/dbw/command:ackermann_msgs/msg/AckermannDriveStamped",
                ],
            )

            self.assertEqual(output_path, (repo_root / "profiles" / "jeep.json").resolve())
            self.assertEqual(notes, [])

            payload = json.loads(output_path.read_text())
            self.assertEqual(payload["name"], "jeep")
            self.assertEqual(payload["extends"], "vehicle")
            self.assertIn("dbw_cmd", payload["required_roles"])
            self.assertEqual(payload["required_sensor_roles"], ["scan"])
            self.assertEqual(payload["resource_hints"]["odom"][0], "/odometry/filtered")
            self.assertEqual(payload["resource_hints"]["dbw_cmd"][0], "/vehicle/dbw/command")
            self.assertEqual(payload["expected_topic_types"]["dbw_cmd"], ["ackermann_msgs/msg/AckermannDriveStamped"])
            self.assertEqual(payload["host_checks"][0]["command"][0], "nvidia-smi")

    def test_scaffold_adds_review_notes_when_dbw_is_missing(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            repo_root = Path(temp_dir)
            output_path, notes = scaffold_profile(
                "jeep",
                repo_root=repo_root,
                topics=[
                    "/map:nav_msgs/msg/OccupancyGrid",
                    "/odom:nav_msgs/msg/Odometry",
                    "/tf:tf2_msgs/msg/TFMessage",
                    "/tf_static:tf2_msgs/msg/TFMessage",
                    "/scan:sensor_msgs/msg/LaserScan",
                ],
                require_nvidia=False,
            )

            payload = json.loads(output_path.read_text())
            self.assertIn("No DBW command topic was auto-detected. Review resource_hints.dbw_cmd.", notes)
            self.assertIn("notes", payload)
            self.assertNotIn("host_checks", payload)


if __name__ == "__main__":
    unittest.main()
