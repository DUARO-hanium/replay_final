#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint_1"]
DXL_RESOURCES = ["dxl11", "dxl12", "dxl13", "dxl14", "dxl15", "dxl16"]
CURRENT_AMPS_PER_RAW = 0.00269
VOLTAGE_VOLTS_PER_RAW = 0.1


class DualFollowerReplayer(Node):
    def __init__(self):
        super().__init__("omx_dual_follower_replayer")
        self.declare_parameter("input_file", "trajectory.csv")
        self.declare_parameter("metrics_file", "dual_replay_metrics.csv")
        self.input_file = self.get_parameter("input_file").value
        self.metrics_path = self.get_parameter("metrics_file").value

        self.publishers = [
            self.create_publisher(
                JointTrajectory, f"/{name}/replay_controller/joint_trajectory", 10
            )
            for name in ("follower1", "follower2")
        ]
        self.latest_states = {"follower1": None, "follower2": None}
        self.subscriptions = [
            self.create_subscription(
                DynamicJointState,
                f"/{name}/dynamic_joint_states",
                lambda msg, robot=name: self.state_callback(robot, msg),
                10,
            )
            for name in ("follower1", "follower2")
        ]
        self.metrics_file = None
        self.metrics_writer = None
        self.metrics_start = None
        self.metrics_count = 0
        self.timer = self.create_timer(1.0 / 30.0, self.metrics_callback)

    def state_callback(self, robot, msg):
        self.latest_states[robot] = {
            resource: dict(zip(interfaces.interface_names, interfaces.values))
            for resource, interfaces in zip(msg.joint_names, msg.interface_values)
        }

    def start_metrics(self):
        self.metrics_file = open(self.metrics_path, "w", newline="")
        self.metrics_writer = csv.writer(self.metrics_file)
        header = ["time"]
        for robot in ("follower1", "follower2"):
            for joint in JOINTS:
                header.extend([
                    f"{robot}_{joint}_current_raw",
                    f"{robot}_{joint}_current_a",
                    f"{robot}_{joint}_voltage_v",
                    f"{robot}_{joint}_power_w",
                ])
            header.append(f"{robot}_total_power_w")
        self.metrics_writer.writerow(header)
        self.metrics_start = time.monotonic()

    def metrics_callback(self):
        if self.metrics_writer is None:
            return
        row = [time.monotonic() - self.metrics_start]
        for robot in ("follower1", "follower2"):
            states = self.latest_states[robot] or {}
            total_power = 0.0
            has_power = False
            for resource in DXL_RESOURCES:
                values = states.get(resource, {})
                current_raw = values.get("Present Current", math.nan)
                voltage_raw = values.get("Present Input Voltage", math.nan)
                current = current_raw * CURRENT_AMPS_PER_RAW if math.isfinite(current_raw) else math.nan
                voltage = voltage_raw * VOLTAGE_VOLTS_PER_RAW if math.isfinite(voltage_raw) else math.nan
                power = voltage * current if math.isfinite(voltage) and math.isfinite(current) else math.nan
                row.extend([current_raw, current, voltage, power])
                if math.isfinite(power):
                    total_power += power
                    has_power = True
            row.append(total_power if has_power else math.nan)
        self.metrics_writer.writerow(row)
        self.metrics_count += 1
        if self.metrics_count % 30 == 0:
            self.metrics_file.flush()

    def load_trajectory(self):
        rows = []
        with open(self.input_file, "r", newline="") as file:
            for row in csv.DictReader(file):
                rows.append((float(row["time"]), [float(row[j]) for j in JOINTS]))
        if not rows:
            raise RuntimeError(f"{self.input_file} is empty")
        first_time = rows[0][0]
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        for stamp, positions in rows:
            relative = stamp - first_time
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = int(relative)
            point.time_from_start.nanosec = int((relative - int(relative)) * 1_000_000_000)
            msg.points.append(point)
        return msg, rows[-1][0] - first_time

    def run(self):
        trajectory, duration = self.load_trajectory()
        self.get_logger().warn(
            "Both followers will move. Clear both workspaces and prepare emergency stops."
        )
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(pub.get_subscription_count() > 0 for pub in self.publishers):
                break
        if not all(pub.get_subscription_count() > 0 for pub in self.publishers):
            raise RuntimeError("Both replay controllers are not connected")

        self.start_metrics()
        for publisher in self.publishers:
            publisher.publish(trajectory)
        self.get_logger().info(f"Sent {len(trajectory.points)} points to both followers")

        end = time.monotonic() + duration + 2.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(
            f"Replay finished; saved {self.metrics_count} rows to {self.metrics_path}"
        )

    def destroy_node(self):
        if self.metrics_file is not None:
            self.metrics_file.flush()
            self.metrics_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DualFollowerReplayer()
    try:
        node.run()
    except (KeyboardInterrupt, RuntimeError) as error:
        if str(error):
            node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
