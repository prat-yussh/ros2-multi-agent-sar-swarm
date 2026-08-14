import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleWanderer(Node):
    def __init__(self):
        super().__init__('simple_wanderer')
        self.cmd_pub = self.create_publisher(Twist, '/robot_1/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/robot_1/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Wanderer Node Active: Exploring the hospital autonomously!')

    def scan_callback(self, msg):
        # Filter out invalid laser readings
        valid_ranges = [r for r in msg.ranges if not float('inf') == r and not float('nan') == r and r > 0.1]
        
        if not valid_ranges:
            return

        # Check the front sector (the middle slice of laser readings)
        total_samples = len(msg.ranges)
        front_samples = msg.ranges[int(total_samples*0.4) : int(total_samples*0.6)]
        valid_front = [r for r in front_samples if r > 0.1]
        
        min_front_dist = min(valid_front) if valid_front else 10.0

        move_cmd = Twist()
        
        # Obstacle avoidance logic
        if min_front_dist < 1.0:
            # Wall detected within 1 meter -> Stop and turn
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5
        else:
            # Path clear -> Drive forward
            move_cmd.linear.x = 0.3
            move_cmd.angular.z = 0.0

        self.cmd_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleWanderer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
