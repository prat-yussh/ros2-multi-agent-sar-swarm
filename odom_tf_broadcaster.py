import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, '/robot_1/odom', self.handle_odometry, 10)
        self.get_logger().info('!!! MASTER SKELETON SYNCED !!!')

    def handle_odometry(self, msg):
        # 1. Wheels to Body
        t1 = TransformStamped()
        t1.header.stamp = msg.header.stamp
        t1.header.frame_id = 'robot_1/odom'
        t1.child_frame_id = 'robot_1/base_link'
        t1.transform.translation.x = msg.pose.pose.position.x
        t1.transform.translation.y = msg.pose.pose.position.y
        t1.transform.translation.z = msg.pose.pose.position.z
        t1.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t1)

        # 2. Body to Laser (Using the exact same timestamp)
        t2 = TransformStamped()
        t2.header.stamp = msg.header.stamp
        t2.header.frame_id = 'robot_1/base_link'
        t2.child_frame_id = 'robot_1/laser_frame'
        t2.transform.translation.x = 0.15
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
