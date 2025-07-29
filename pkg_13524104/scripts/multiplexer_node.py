#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SmartMultiplexer(Node):
    def __init__(self):
        super().__init__('smart_multiplexer')

        # Inisialisasi pesan terakhir
        self.keyboard_msg = None
        self.joy_msg = None
        self.auto_msg = None

        # Subscriber untuk masing-masing sumber
        self.create_subscription(Twist, '/keyboard_vel', self.keyboard_callback, 10)
        self.create_subscription(Twist, '/joy_vel', self.joy_callback, 10)
        self.create_subscription(Twist, '/autonomous_vel', self.autonomous_callback, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_type_pub = self.create_publisher(String, '/cmd_type', 10)

        # Timer 5Hz
        timer_period = 0.2  # 5 Hz (t = 1/f (1/5))
        self.create_timer(timer_period, self.highest_priority)

        self.get_logger().info("Smart Multiplexer started!")
        self.get_logger().info("Priority Order: Keyboard > Joy > Autonomous")

    def keyboard_callback(self, msg):
        self.keyboard_msg = msg

    def joy_callback(self, msg):
        self.joy_msg = msg

    def autonomous_callback(self, msg):
        self.auto_msg = msg

    def highest_priority(self):
        if self.keyboard_msg is not None:
            self.publish(self.keyboard_msg, 'keyboard')
            self.keyboard_msg = None
        elif self.joy_msg is not None:
            self.publish(self.joy_msg, 'joy')
            self.joy_msg = None
        elif self.auto_msg is not None:
            self.publish(self.auto_msg, 'autonomous')
            self.auto_msg = None
        else:
            # Tidak ada input saat ini
            pass

    def publish(self, twist_msg, source_str):
        # Publish Twist
        self.cmd_vel_pub.publish(twist_msg)

        # Publish cmd_type
        source = String()
        source.data = source_str
        self.cmd_type_pub.publish(source)

        # Logging (Memberi tahu sumber yang sedang aktif (kapital) dengan nilai twis-nya)
        self.get_logger().info(
            f"[{source_str.upper()}] ➤ Lin.x = {twist_msg.linear.x:.2f} m/s, "
            f"Ang.z = {twist_msg.angular.z:.2f} rad/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SmartMultiplexer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
