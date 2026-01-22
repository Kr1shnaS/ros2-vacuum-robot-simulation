#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.listener_callback, 10)
        self.twist = Twist()
    def listener_callback(self, msg):
        scan_range = [r for r in msg.ranges[150:210] if r > 0.1]
        if not scan_range:
            return
        min_distance = min(scan_range)
        self.get_logger().info(f'Distance to cone: {min_distance:.2f}m')
        if min_distance < 0.6:
            self.get_logger().warn('CONE DETECTED! Turning...')
            self.twist.linear.x = 0.0   
            self.twist.angular.z = 0.5  
        else:
            self.twist.linear.x = 0.2   
            self.twist.angular.z = 0.0  
        self.publisher.publish(self.twist)
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
