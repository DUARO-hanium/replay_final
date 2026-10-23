#!/usr/bin/env python3

import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "gripper_joint_1",
]

RECORD_HZ = 30.0
RECORD_PERIOD = 1.0 / RECORD_HZ


class Recorder(Node):

    def __init__(self):
        super().__init__("omx_trajectory_recorder")

        self.declare_parameter("joint_states_topic", "/leader/joint_states")
        self.declare_parameter("output_file", "trajectory.csv")
        self.joint_states_topic = self.get_parameter(
            "joint_states_topic"
        ).value
        self.output_file = self.get_parameter("output_file").value

        # 가장 최근 joint 값
        self.latest_positions = None
        self.last_state_time = None

        # joint state 수신
        self.sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self.joint_callback,
            qos_profile_sensor_data,
        )

        # CSV 파일
        self.file = open(
            self.output_file,
            "w",
            newline="",
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow(
            ["time"] + JOINTS
        )

        # 녹화 시작 시간
        self.start_time = time.monotonic()

        self.count = 0

        # 30 Hz timer
        self.timer = self.create_timer(
            RECORD_PERIOD,
            self.record_timer_callback,
        )

        self.get_logger().info(
            f"Recording {self.joint_states_topic} to {self.output_file} "
            f"at {RECORD_HZ:.1f} Hz."
        )

    def joint_callback(self, msg):
        """
        joint_states가 들어오면
        CSV에 바로 저장하지 않고 최신 joint 값만 갱신한다.
        """

        states = dict(
            zip(msg.name, msg.position)
        )

        if not all(
            joint in states and math.isfinite(states[joint])
            for joint in JOINTS
        ):
            return

        self.latest_positions = [
            states[joint]
            for joint in JOINTS
        ]
        self.last_state_time = time.monotonic()

    def record_timer_callback(self):
        """
        정확히 30 Hz timer 기준으로
        가장 최근 joint 값을 CSV에 기록한다.
        """

        if self.latest_positions is None:
            return

        if time.monotonic() - self.last_state_time > 0.5:
            self.get_logger().error("Joint states stopped; recording ended to avoid stale data.")
            raise RuntimeError("No fresh joint states for 0.5 seconds")

        elapsed = (
            time.monotonic()
            - self.start_time
        )

        self.writer.writerow(
            [elapsed]
            + self.latest_positions
        )

        self.count += 1

        # 약 1초마다 flush + log
        if self.count % int(RECORD_HZ) == 0:

            self.file.flush()

            self.get_logger().info(
                f"Recorded {self.count} samples "
                f"({elapsed:.2f} sec)"
            )

    def destroy_node(self):

        if not self.file.closed:
            self.file.flush()
            self.file.close()

        duration = (
            time.monotonic()
            - self.start_time
        )

        actual_hz = (
            self.count / duration
            if duration > 0
            else 0.0
        )

        self.get_logger().info(
            f"Saved {self.count} samples "
            f"to {self.output_file}"
        )

        self.get_logger().info(
            f"Duration: {duration:.2f} sec, "
            f"average record rate: {actual_hz:.2f} Hz"
        )

        super().destroy_node()


def main():

    rclpy.init()

    node = Recorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
