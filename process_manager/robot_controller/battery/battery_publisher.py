#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import BatteryInfo, BmsStatus
import random


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.battery_info_pub = self.create_publisher(BatteryInfo, '/battery_info', 10)
        self.bms_status_pub = self.create_publisher(BmsStatus, '/bms_status', 10)

        # Initial simulated battery state
        self.charge_pct = 100.0
        self.current = 5.0  # Amps
        self.temperature = 30.0  # Celsius
        self.capacity = 5000  # mAh

        self.cell_voltage_nominal = 3.7
        self.cell_voltages = [self.cell_voltage_nominal for _ in range(7)]
        self.fault_bits = [0 for _ in range(8)]

        self.timer = self.create_timer(2.0, self.publish_battery_data)  # 2s interval

    def publish_battery_data(self):
        # Simulate discharge, reset when empty
        self.charge_pct -= 0.5
        if self.charge_pct <= 0.0:
            self.charge_pct = 100.0  # 🔁 Reset charge after empty

        # Realistic noise
        self.current += random.uniform(-0.2, 0.2)
        self.temperature += random.uniform(-0.1, 0.1)
        self.cell_voltages = [
            max(3.0, v - 0.002 + random.uniform(-0.002, 0.002)) for v in self.cell_voltages
        ]

        # ---- BMS Status ----
        bms_msg = BmsStatus()
        bms_msg.temps = [int(self.temperature + random.uniform(-1.5, 1.5)) for _ in range(2)]
        bms_msg.cell_voltages = [int(v * 1000) for v in self.cell_voltages]
        bms_msg.charge_state = 1 if self.charge_pct > 20 else 0
        bms_msg.fault_bits = self.fault_bits.copy()
        self.bms_status_pub.publish(bms_msg)

        # ---- Battery Info ----
        batt_msg = BatteryInfo()
        batt_msg.total_voltage = float(sum(self.cell_voltages))
        batt_msg.measured_voltage = batt_msg.total_voltage + random.uniform(-0.05, 0.05)
        batt_msg.current = float(self.current)
        batt_msg.soc = float(self.charge_pct)
        batt_msg.capacity = int(self.capacity)
        self.battery_info_pub.publish(batt_msg)

        self.get_logger().info(
            f"🔋 BatteryInfo: {batt_msg.soc:.1f}%, {batt_msg.current:.2f}A, {batt_msg.total_voltage:.2f}V | "
            f"BMS: {bms_msg.temps[0]}°C, {len(bms_msg.cell_voltages)} cells"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
