import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import math

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

        self.loop_ticks = 0
        self.shift_ticks = 0
        
        self.room_shift_dir = -1.0 
        self.march_yaw = 0.0
        
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
        
        self.victim_x = -6.0
        self.victim_y = -3.0
        
        self.target_yaw = 0.0
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
        if self.state in ["WAITING", "DEPLOY", "FAN_OUT_TURN", "FAN_OUT_DRIVE", "FACE_MARCH"]:
            self.min_front = 10.0
            return
            
        mid = len(msg.ranges) // 2
        front_arc = msg.ranges[mid - 3 : mid + 3] 
        valid_front = [r for r in front_arc if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        self.min_front = min(valid_front) if valid_front else 10.0

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        self.loop_ticks += 1
        cmd = Twist()
        
        dist_to_victim = math.hypot(self.current_x - self.victim_x, self.current_y - self.victim_y)
        if dist_to_victim < 2.0 and self.state not in ["RESCUE", "STANDBY"]:
            self.get_logger().info(f"[{self.name.upper()}] 🚨 VICTIM LOCATED! Broadcasting to Swarm. 🚨")
            self.state = "RESCUE"
            msg = Bool()
            msg.data = True
            self.alert_pub.publish(msg)
        
        if self.state == "WAITING":
            if self.loop_ticks > 60: 
                self.state = "DEPLOY"
                
        elif self.state == "DEPLOY":
            if self.current_y > 2.0: 
                cmd.linear.x = 0.8 
                err = self.normalize_angle(0.0 - self.current_yaw)
                cmd.angular.z = max(-1.0, min(1.0, 4.0 * err))
            else:
                self.state = "FAN_OUT_TURN"
                if self.name == 'robot_2': self.target_yaw = 1.5708
                elif self.name == 'robot_3': self.target_yaw = -1.5708
                else: self.target_yaw = 0.0
                
        elif self.state == "FAN_OUT_TURN":
            cmd.linear.x = 0.0 
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.05: 
                cmd.angular.z = max(-1.5, min(1.5, 4.0 * err))
            else:
                self.state = "FAN_OUT_DRIVE"
                self.shift_ticks = 0

        elif self.state == "FAN_OUT_DRIVE":
            cmd.linear.x = 0.8 if self.name != 'robot_1' else 0.0 
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            cmd.angular.z = max(-1.0, min(1.0, 4.0 * err))
            self.shift_ticks += 1
            if self.shift_ticks > 35: 
                self.state = "FACE_MARCH"
                self.target_yaw = 0.0 

        elif self.state == "FACE_MARCH":
            cmd.linear.x = 0.0
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.05:
                cmd.angular.z = max(-1.5, min(1.5, 4.0 * err))
            else:
                self.state = "MARCH"

        elif self.state == "MARCH":
            cmd.linear.x = 0.8 
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            cmd.angular.z = max(-1.0, min(1.0, 4.0 * err)) 
            
            # GEOFENCE FIX: Moved up to Y=4.5 so they sweep all the way to the front wall!
            facing_north = abs(self.target_yaw) > 1.5 
            hit_virtual_wall = (self.current_y > 4.5 and facing_north)
            
            if self.min_front < 1.0 or hit_virtual_wall:
                self.state = "SWEEP_TURN_1"
                self.march_yaw = self.current_yaw 
                
                if abs(self.current_yaw) < 1.5: 
                    self.target_yaw = self.normalize_angle(self.current_yaw + (self.room_shift_dir * 1.5708))
                else: 
                    self.target_yaw = self.normalize_angle(self.current_yaw - (self.room_shift_dir * 1.5708))

        elif self.state == "SWEEP_TURN_1":
            cmd.linear.x = 0.0
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.05:
                cmd.angular.z = max(-1.5, min(1.5, 4.0 * err))
            else:
                self.state = "SWEEP_SHIFT" 
                self.shift_ticks = 0

        elif self.state == "SWEEP_SHIFT":
            cmd.linear.x = 0.6
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            cmd.angular.z = max(-1.0, min(1.0, 4.0 * err))
            self.shift_ticks += 1
            
            if self.shift_ticks > 30 or self.min_front < 0.8:
                if self.min_front < 0.8:
                    self.room_shift_dir *= -1.0 
                    
                self.state = "SWEEP_TURN_2"
                self.target_yaw = self.normalize_angle(self.march_yaw + 3.14159) 

        elif self.state == "SWEEP_TURN_2":
            cmd.linear.x = 0.0
            err = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(err) > 0.05:
                cmd.angular.z = max(-1.5, min(1.5, 4.0 * err))
            else:
                self.state = "MARCH" 
                
        elif self.state in ["RESCUE", "STANDBY"]:
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
