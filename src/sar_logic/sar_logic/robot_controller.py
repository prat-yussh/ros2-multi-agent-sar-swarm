import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import math
import time

class MathematicalSwarm(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.declare_parameter('robot_name', 'robot_1')
        self.name = self.get_parameter('robot_name').value

        self.cmd_pub = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.name}/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, f'/{self.name}/odom', self.odom_callback, 10)

        self.alert_pub = self.create_publisher(Bool, '/rescue_alert', 10)
        self.alert_sub = self.create_subscription(Bool, '/rescue_alert', self.alert_callback, 10)

        self.start_time = time.time()
        if self.name == 'robot_1':
            self.state = "DEPLOY"
            self.get_logger().info("[LEADER] Tactical Breach Initiated.")
        else:
            self.state = "WAITING"
        
        self.start_y = 12.0
        self.start_x = 0.0
        if self.name == 'robot_2': self.start_x = 0.7
        elif self.name == 'robot_3': self.start_x = -0.7
        
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_yaw = 0.0 
        self.min_front = 10.0
        
        self.victim_x = 3.0
        self.victim_y = -4.0
        
        self.sweep_dir = -1.0 if self.name == 'robot_3' else 1.0 
        self.target_yaw = 0.0
        self.shift_start_time = 0.0

        self.timer = self.create_timer(0.05, self.control_loop) 

    def alert_callback(self, msg):
        if msg.data and self.state not in ["RESCUE", "STANDBY"]:
            self.get_logger().info(f"[{self.name.upper()}] Broadcast received! Standing by.")
            self.state = "STANDBY"

    def odom_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        self.current_y = self.start_y - odom_x
        self.current_x = self.start_x + odom_y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        mid = len(msg.ranges) // 2
        # THE FIX: Extreme tunnel vision for the doorway, wide vision for the room.
        if self.state in ["WAITING", "DEPLOY"]:
            front_arc = msg.ranges[mid - 10 : mid + 10] 
        else:
            front_arc = msg.ranges[mid - 40 : mid + 40] 
            
        valid_front = [r for r in front_arc if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        self.min_front = min(valid_front) if valid_front else 10.0

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        cmd = Twist()
        
        # 🚨 VICTIM DETECTION 🚨
        dist_to_victim = math.sqrt((self.current_x - self.victim_x)**2 + (self.current_y - self.victim_y)**2)
        if dist_to_victim < 2.0 and self.state != "RESCUE":
            self.get_logger().error(f"[{self.name.upper()}] 🚨 VICTIM LOCATED! Broadcasting to Swarm. 🚨")
            self.state = "RESCUE"
            msg = Bool()
            msg.data = True
            self.alert_pub.publish(msg)
        
        # THE FIX: Split emergency avoidance thresholds
        if self.state == "DEPLOY" and self.min_front < 0.8:
            self.state = "TURN_1"
            self.target_yaw = self.normalize_angle(self.current_yaw + (self.sweep_dir * 1.57))
        elif self.state == "SWEEP" and self.min_front < 2.0:
            self.state = "TURN_1"
            self.target_yaw = self.normalize_angle(self.current_yaw + (self.sweep_dir * 1.57))
        
        # --- STATE MACHINE ---
        if self.state == "WAITING":
            if time.time() - self.start_time > 3.0: 
                self.state = "DEPLOY"
                
        elif self.state == "DEPLOY":
            if self.current_y > 2.0: 
                cmd.linear.x = 0.8 # Moderate speed to safely thread the needle through the door
            else:
                self.state = "SWEEP"
                
        elif self.state == "SWEEP":
            # High speed sweep mode!
            cmd.linear.x = 1.5 
                
        elif self.state == "TURN_1":
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.1: cmd.angular.z = 2.5 * err 
            else:
                self.state = "SHIFT"
                self.shift_start_time = time.time()
                
        elif self.state == "SHIFT":
            if time.time() - self.shift_start_time < 1.0: cmd.linear.x = 1.0 
            else:
                self.state = "TURN_2"
                self.target_yaw = self.normalize_angle(self.current_yaw + (self.sweep_dir * 1.57))
                
        elif self.state == "TURN_2":
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.1: cmd.angular.z = 2.5 * err 
            else:
                self.state = "SWEEP"
                self.sweep_dir *= -1.0 
                
        elif self.state == "RESCUE":
            cmd.linear.x = 0.0
            cmd.angular.z = 3.0
            
        elif self.state == "STANDBY":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
                
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MathematicalSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
