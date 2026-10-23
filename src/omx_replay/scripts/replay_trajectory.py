#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "gripper_joint_1",
]
DXL_RESOURCES = [
    "dxl1", "dxl2", "dxl3", "dxl4", "dxl5", "dxl6",
]

# XM430 계열의 Present Current 단위: 2.69 mA / raw count
CURRENT_AMPS_PER_RAW = 0.00269



class Replayer(Node):

    def __init__(self):
        super().__init__("omx_trajectory_replayer")

        self.declare_parameter(
            "trajectory_topic", "/replay_controller/joint_trajectory"
        )
        self.declare_parameter("dynamic_joint_states_topic", "/dynamic_joint_states")
        self.declare_parameter("input_file", "trajectory.csv")
        self.declare_parameter("metrics_file", "replay_metrics.csv")
        self.declare_parameter("dxl_resources", DXL_RESOURCES)
        self.declare_parameter("start_delay_s", 3.0)
        self.trajectory_topic = self.get_parameter("trajectory_topic").value
        self.dynamic_joint_states_topic = self.get_parameter(
            "dynamic_joint_states_topic"
        ).value
        self.input_file = self.get_parameter("input_file").value
        self.metrics_path = self.get_parameter("metrics_file").value
        self.dxl_resources = list(self.get_parameter("dxl_resources").value)
        self.start_delay = float(self.get_parameter("start_delay_s").value)
        if not math.isfinite(self.start_delay) or self.start_delay < 0.1:
            raise ValueError("start_delay_s must be finite and at least 0.1 seconds")
        self.actual_positions = None
        self.actual_time = 0.0
        self.actual_sub = self.create_subscription(
            JointState, "/joint_states", self.actual_callback, qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            self.trajectory_topic,
            10,
        )
        self.state_sub = self.create_subscription(
            DynamicJointState,
            self.dynamic_joint_states_topic,
            self.state_callback,
            10,
        )

        self.metrics_file = None
        self.metrics_writer = None
        self.metrics_start_time = None
        self.metrics_count = 0
        self.missing_interfaces_warned = False

    def actual_callback(self, msg):
        states = dict(zip(msg.name, msg.position))
        if all(j in states and math.isfinite(states[j]) for j in JOINTS):
            self.actual_positions = [states[j] for j in JOINTS]
            self.actual_time = time.monotonic()

    def start_metrics_recording(self):
        self.metrics_file = open(self.metrics_path, "w", newline="")
        self.metrics_writer = csv.writer(self.metrics_file)

        header = ["time"]
        for joint in JOINTS:
            header.extend([
                f"{joint}_current_raw",
                f"{joint}_current_a",
                f"{joint}_voltage_v",
                f"{joint}_power_w",
            ])
        header.append("total_power_w")

        self.metrics_writer.writerow(header)
        self.metrics_start_time = time.monotonic()

    def state_callback(self, msg):
        if self.metrics_writer is None:
            return

        states = {}
        for resource, interfaces in zip(msg.joint_names, msg.interface_values):
            states[resource] = dict(zip(
                interfaces.interface_names,
                interfaces.values,
            ))

        row = [time.monotonic() - self.metrics_start_time]
        total_power = 0.0
        has_power = False

        for resource in self.dxl_resources:
            values = states.get(resource, {})
            current_raw = values.get("Present Current", math.nan)
            voltage = values.get("Present Input Voltage", math.nan)
            current = (
                current_raw * CURRENT_AMPS_PER_RAW
                if math.isfinite(current_raw)
                else math.nan
            )
            power = (
                voltage * current
                if math.isfinite(voltage) and math.isfinite(current)
                else math.nan
            )

            row.extend([current_raw, current, voltage, power])
            if math.isfinite(power):
                total_power += power
                has_power = True

        row.append(total_power if has_power else math.nan)
        self.metrics_writer.writerow(row)
        self.metrics_count += 1

        if self.metrics_count % 30 == 0:
            self.metrics_file.flush()

        if not has_power and not self.missing_interfaces_warned:
            self.get_logger().warn(
                "Present Current/Present Input Voltage interfaces "
                f"are not available in {self.dynamic_joint_states_topic}."
            )
            self.missing_interfaces_warned = True


    def load_csv(self, path):
        rows = []

        with open(path, "r") as f:
            reader = csv.DictReader(f)

            for row in reader:
                sample = {
                    "time": float(row["time"]),
                    "positions": [
                        float(row[joint])
                        for joint in JOINTS
                    ],
                }
                if not all(math.isfinite(v) for v in [sample["time"], *sample["positions"]]):
                    raise ValueError("CSV contains a nonfinite time or joint position")
                if rows and sample["time"] <= rows[-1]["time"]:
                    raise ValueError("CSV timestamps must be strictly increasing")
                rows.append(sample)

        return rows

    def make_trajectory(self, recorded):
        msg = JointTrajectory()
        msg.joint_names = JOINTS

        if self.actual_positions is None:
            raise RuntimeError("Current joint states are required before replay")
        initial = JointTrajectoryPoint()
        initial.positions = list(self.actual_positions)
        initial.time_from_start.nanosec = 100_000_000
        msg.points.append(initial)

        # CSV의 첫 timestamp를 0초 기준으로 맞춤
        first_time = recorded[0]["time"]

        for row in recorded:

            relative_time = row["time"] - first_time + self.start_delay

            point = JointTrajectoryPoint()

            # CSV에 기록된 joint position 그대로 사용
            point.positions = row["positions"]

            sec = int(relative_time)
            nsec = int(
                (relative_time - sec) * 1_000_000_000
            )

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nsec

            msg.points.append(point)

        return msg

    def run(self):

        recorded = self.load_csv(self.input_file)

        if not recorded:
            self.get_logger().error(
                f"{self.input_file} is empty."
            )
            return

        duration = (
            recorded[-1]["time"]
            - recorded[0]["time"]
        )

        self.get_logger().info(
            f"Loaded {len(recorded)} samples"
        )

        self.get_logger().info(
            f"Recorded motion duration: {duration:.2f} sec"
        )

        self.get_logger().warn(
            f"Robot will approach the first pose over {self.start_delay:.1f}s, "
            f"then replay {self.input_file}."
        )

        # Wait for a subscriber and a current pose; do not silently drop the trajectory.
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.pub.get_subscription_count() > 0 and
                    self.actual_positions is not None and
                    time.monotonic() - self.actual_time < 0.5):
                break
        else:
            raise RuntimeError("Replay controller subscriber or fresh /joint_states unavailable")
        if not rclpy.ok():
            return
        msg = self.make_trajectory(recorded)
        self.start_metrics_recording()

        self.pub.publish(msg)

        self.get_logger().info(
            "Trajectory sent. Recording voltage, current, and power "
            f"to {self.metrics_path}."
        )

        # trajectory가 끝날 때까지 node 유지
        end_time = (
            time.monotonic()
            + duration
            + self.start_delay
            + 2.0
        )

        while (
            rclpy.ok()
            and time.monotonic() < end_time
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        self.get_logger().info(
            f"Replay finished. Saved {self.metrics_count} power samples "
            f"to {self.metrics_path}."
        )

    def destroy_node(self):
        if self.metrics_file is not None:
            self.metrics_file.flush()
            self.metrics_file.close()
            self.metrics_file = None

        super().destroy_node()


def main():

    rclpy.init()

    node = Replayer()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
