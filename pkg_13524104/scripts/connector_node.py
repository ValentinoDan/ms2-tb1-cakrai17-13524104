#!/usr/bin/env python3
# ^ untuk menjalankan file python secara langsung

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ConnectorNode(Node):
    def __init__(self):
        super().__init__('connector_node')
        self.subscription = self.create_subscription(
            Twist,
            'autonomous_vel',
            self.listener_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_type_pub = self.create_publisher(String, 'cmd_type', 10)

    def listener_callback(self, msg):
        self.cmd_vel_pub.publish(msg)
        cmd_type_msg = String()
        cmd_type_msg.data = 'autonomous'
        self.cmd_type_pub.publish(cmd_type_msg)
        self.get_logger().info(f'Translated autonomous_vel to cmd_vel. x={msg.linear.x}, z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args) # inisialisasi ROS
    node = ConnectorNode()
    rclpy.spin(node) # Membuat node tetap berjalan
    node.destroy_node()
    rclpy.shutdown() # shutdown

if __name__ == '__main__':
    main()
