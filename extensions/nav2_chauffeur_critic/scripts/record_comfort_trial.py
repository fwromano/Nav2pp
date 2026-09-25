#!/usr/bin/env python3
"""Record synchronized-enough vehicle comfort telemetry to CSV.

This logger intentionally records raw observables rather than calculating the
comfort score online. Analyze the resulting CSV offline with replay_vehicle_log.py.

Default assumptions:
- odometry twist is expressed in the vehicle/base frame;
- IMU +Y is vehicle-left lateral acceleration;
- IMU frame is sufficiently aligned with base_link for the experiment.

For publication-quality testing, verify frame alignment and time synchronization.
"""

import csv
import pathlib
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ComfortTrialRecorder(Node):
    def __init__(self):
        super().__init__("comfort_trial_recorder")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("output_csv", "comfort_trial.csv")
        self.declare_parameter("sample_rate_hz", 50.0)
        self.declare_parameter("lateral_accel_sign", 1.0)

        odom_topic = self.get_parameter("odom_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        output = pathlib.Path(self.get_parameter("output_csv").value)
        hz = float(self.get_parameter("sample_rate_hz").value)
        self.ay_sign = float(self.get_parameter("lateral_accel_sign").value)

        output.parent.mkdir(parents=True, exist_ok=True)
        self.file = output.open("w", newline="")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "time_s",
                "ros_time_s",
                "vx_mps",
                "vy_mps",
                "wz_rad_s",
                "imu_ax_mps2",
                "ay_mps2",
                "imu_az_mps2",
                "imu_wz_rad_s",
            ],
        )
        self.writer.writeheader()

        self.start_monotonic = time.monotonic()
        self.odom = None
        self.imu = None

        self.create_subscription(Odometry, odom_topic, self._odom, 50)
        self.create_subscription(Imu, imu_topic, self._imu, 100)
        self.timer = self.create_timer(1.0 / hz, self._sample)

        self.get_logger().info(
            f"Recording {odom_topic} + {imu_topic} at {hz:.1f} Hz to {output}"
        )

    def _odom(self, msg):
        self.odom = msg

    def _imu(self, msg):
        self.imu = msg

    def _sample(self):
        if self.odom is None or self.imu is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        row = {
            "time_s": time.monotonic() - self.start_monotonic,
            "ros_time_s": now,
            "vx_mps": self.odom.twist.twist.linear.x,
            "vy_mps": self.odom.twist.twist.linear.y,
            "wz_rad_s": self.odom.twist.twist.angular.z,
            "imu_ax_mps2": self.imu.linear_acceleration.x,
            "ay_mps2": self.ay_sign * self.imu.linear_acceleration.y,
            "imu_az_mps2": self.imu.linear_acceleration.z,
            "imu_wz_rad_s": self.imu.angular_velocity.z,
        }
        self.writer.writerow(row)
        self.file.flush()

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = ComfortTrialRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
