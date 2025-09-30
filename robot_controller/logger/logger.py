#!/usr/bin/env python3
"""
logger.py

Run this alongside your ROS 2/Django stack to record every incoming message on
the listed topics into separate CSVs under ./logs/.

Each topic → one CSV named <topicname>_YYYYMMDD_HHMMSS.csv
"""

import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node

# Standard ROS 2 message types
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# Your custom messages (as imported by views.py)
from custom_msgs.msg import (
    DrivetrainFeedback,
    ScienceFeedback,
    RadioFeedback,
    BatteryFeedback,
    CoreFeedback,
)

# ───────────────────────────────────────────────────────────────────────────────
# EDIT THIS SECTION TO MATCH YOUR ACTUAL TOPICS & TYPES
# We omit /camera_* (image_compressed) because those carry binary JPEGs.
# ───────────────────────────────────────────────────────────────────────────────
TOPIC_CONFIG = {
    # Arm topics
    "/arm_command": JointState,
    "/arm_velocity_command": JointState,

    # Battery feedback
    "/battery_feedback": BatteryFeedback,

    # Drivetrain feedback
    "/drivetrain_feedback": DrivetrainFeedback,

    # Core feedback
    "/core_feedback": CoreFeedback,

    # Radio feedback
    "/radio_feedback": RadioFeedback,

    # Science feedback
    "/science_feedback": ScienceFeedback,

    # Rover logs (String messages)
    "/rover_logs": String,
}

LOG_DIR = "logs"


class MultiTopicLogger(Node):
    def __init__(self, output_dir=LOG_DIR):
        super().__init__("multi_topic_logger")

        # 1) Ensure the output_dir exists
        os.makedirs(output_dir, exist_ok=True)

        # 2) For each topic, open one CSV file and write headers
        self.loggers = {}         # topic_name → (file_obj, csv_writer)
        self._subscriptions = []  # keep subscriber refs alive

        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        for topic_name, msg_type in TOPIC_CONFIG.items():
            # Create a “safe” filename (replace slashes with underscores)
            safe_name = topic_name.strip("/").replace("/", "_") or "root"
            filename = os.path.join(output_dir, f"{safe_name}_{now_str}.csv")

            f = open(filename, "w", newline="")
            writer = csv.writer(f)

            # Write header row depending on msg_type
            if msg_type is JointState:
                header = ["timestamp"]
                for i in range(6):
                    header.append(f"joint_{i}_pos")
                for i in range(6):
                    header.append(f"joint_{i}_vel")

            elif msg_type is DrivetrainFeedback:
                # DrivetrainFeedback: epoch_time, wheel_position (list), wheel_velocity (list), wheel_torque (list)
                header = ["timestamp", "epoch_time"]
                for i in range(4):
                    header.append(f"wheel_{i}_pos")
                for i in range(4):
                    header.append(f"wheel_{i}_vel")
                for i in range(4):
                    header.append(f"wheel_{i}_torque")

            elif msg_type is CoreFeedback:
                # CoreFeedback: epoch_time, wheel_position (list), wheel_velocity, wheel_torque, pitch, roll
                header = ["timestamp", "epoch_time"]
                for i in range(4):
                    header.append(f"wheel_{i}_pos")
                for i in range(4):
                    header.append(f"wheel_{i}_vel")
                for i in range(4):
                    header.append(f"wheel_{i}_torque")
                header.extend(["pitch", "roll"])

            elif msg_type is ScienceFeedback:
                # ScienceFeedback: rfid, moisture, potentiometer, limit, height
                header = ["timestamp", "rfid", "moisture", "potentiometer", "limit", "height"]

            elif msg_type is RadioFeedback:
                # RadioFeedback: signal_strength, ping_ms, rx_bytes, tx_bytes
                header = ["timestamp", "signal_strength", "ping_ms", "rx_bytes", "tx_bytes"]

            elif msg_type is BatteryFeedback:
                # BatteryFeedback: charge_pct, current_draw, temperature, timestamp
                header = ["timestamp", "charge_pct", "current_draw", "temperature", "battery_timestamp"]

            elif msg_type is String:
                header = ["timestamp", "data"]

            else:
                # Fallback: store entire message as one string
                header = ["timestamp", "message"]

            writer.writerow(header)
            f.flush()

            self.loggers[topic_name] = (f, writer)

            # Create the subscription, capturing topic_name in a default argument
            sub = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, tn=topic_name: self._generic_callback(tn, msg),
                10,
            )
            self._subscriptions.append(sub)
            self.get_logger().info(f"Logging topic {topic_name} → {filename}")

    def _generic_callback(self, topic_name, msg):
        """
        Called whenever a new message arrives on topic_name.
        We look up which CSV writer to use and append one row:
          [timestamp, <flattened fields>]
        """
        now = self.get_clock().now().to_msg()
        timestamp = f"{now.sec}.{now.nanosec:09d}"

        f, writer = self.loggers[topic_name]

        # Dispatch based on message type
        if isinstance(msg, JointState):
            positions = list(msg.position)[:6] + [0.0] * max(0, 6 - len(msg.position))
            velocities = list(msg.velocity)[:6] + [0.0] * max(0, 6 - len(msg.velocity))
            row = [timestamp] + positions + velocities

        elif isinstance(msg, DrivetrainFeedback):
            row = [timestamp, msg.epoch_time]
            wpos = list(msg.wheel_position)[:4] + [0.0] * max(0, 4 - len(msg.wheel_position))
            wvel = list(msg.wheel_velocity)[:4] + [0.0] * max(0, 4 - len(msg.wheel_velocity))
            wtor = list(msg.wheel_torque)[:4] + [0.0] * max(0, 4 - len(msg.wheel_torque))
            row.extend(wpos + wvel + wtor)

        elif isinstance(msg, CoreFeedback):
            row = [timestamp, int(msg.epoch_time)]
            wpos = [float(x) for x in msg.wheel_position][:4] + [0.0] * max(0, 4 - len(msg.wheel_position))
            wvel = [float(x) for x in msg.wheel_velocity][:4] + [0.0] * max(0, 4 - len(msg.wheel_velocity))
            wtor = [float(x) for x in msg.wheel_torque][:4] + [0.0] * max(0, 4 - len(msg.wheel_torque))
            row.extend(wpos + wvel + wtor)
            row.extend([round(float(msg.pitch), 2), round(float(msg.roll), 2)])

        elif isinstance(msg, ScienceFeedback):
            row = [
                timestamp,
                msg.rfid,
                msg.moisture,
                msg.potentiometer,
                msg.limit,
                msg.height,
            ]

        elif isinstance(msg, RadioFeedback):
            row = [
                timestamp,
                round(msg.signal_strength, 2),
                msg.ping_ms,
                msg.rx_bytes,
                msg.tx_bytes,
            ]

        elif isinstance(msg, BatteryFeedback):
            row = [
                timestamp,
                round(msg.charge_pct, 2),
                round(msg.current_draw, 2),
                round(msg.temperature, 2),
                msg.timestamp,
            ]

        elif isinstance(msg, String):
            text = msg.data.replace("\n", " ").replace("\r", " ")
            row = [timestamp, text]

        else:
            row = [timestamp, str(msg)]

        writer.writerow(row)
        f.flush()

    def destroy_node(self):
        # Close all CSV files on shutdown
        for f, _ in self.loggers.values():
            f.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicLogger(output_dir=LOG_DIR)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
