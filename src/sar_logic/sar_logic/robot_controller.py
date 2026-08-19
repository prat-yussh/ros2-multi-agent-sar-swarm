import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import math
import random
import subprocess
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

        # Per-robot lane bounds: 30m room split into 3x lanes, 1m wall margin
        if self.name == 'robot_2':
            self.lane_min_x, self.lane_max_x = 5.0, 14.0
        elif self.name == 'robot_3':
            self.lane_min_x, self.lane_max_x = -14.0, -5.0
        else:
            self.lane_min_x, self.lane_max_x = -5.0, 5.0

        # Victim broadcast (robot_1 generates + republishes on a 1s timer)
        self.victim_pub = self.create_publisher(Point, '/victim_location', 10)
        self.victim_sub = self.create_subscription(Point, '/victim_location', self.victim_callback, 10)
        self.victim_spawned = False
        self.victim_broadcast_timer = self.create_timer(1.0, self.broadcast_victim)

        # Rescue extraction: (1) retreat north on own lane, (2) pinch to gate X, (3) exit
        if self.name == 'robot_2':
            self.rescue_waypoints = [(9.5, 4.0), (0.7, 4.0), (0.7, 10.0), (0.0, 11.0)]
        elif self.name == 'robot_3':
            self.rescue_waypoints = [(-9.5, 4.0), (-0.7, 4.0), (-0.7, 10.0), (0.0, 11.0)]
        else:
            self.rescue_waypoints = [(0.0, 4.0), (0.0, 10.0)]
        self.rescue_phase = 0
        self.rescue_stuck_ticks = 0

        self.loop_ticks = 0
        self.shift_ticks = 0

        self.room_shift_dir = 1.0 if self.name == 'robot_2' else -1.0
        self.march_yaw = 0.0
        self.target_yaw = 0.0

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

        self.victim_x = 999.0
        self.victim_y = 999.0

        self.timer = self.create_timer(0.05, self.control_loop)

    def victim_callback(self, msg):
        self.victim_x = msg.x
        self.victim_y = msg.y

    def broadcast_victim(self):
        if self.name == 'robot_1' and self.victim_spawned:
            msg = Point()
            msg.x, msg.y = self.victim_x, self.victim_y
            self.victim_pub.publish(msg)

    def spawn_victim(self):
        self.victim_x = round(random.uniform(-13.0, 13.0), 2)
        self.victim_y = round(random.uniform(-13.0, 3.0), 2)
        self.get_logger().info(f"[LEADER] Victim placed at ({self.victim_x}, {self.victim_y})")

        victim_name = f"victim_{int(time.time() * 1000)}"
        sdf = f"""<?xml version='1.0'?><sdf version='1.9'><model name='{victim_name}'>
        <static>true</static>
        <link name='link'><visual name='visual'><geometry><box><size>0.4 0.4 0.4</size></box></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link>
        </model></sdf>"""
        subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-string', sdf, '-name', victim_name,
            '-x', str(self.victim_x), '-y', str(self.victim_y), '-z', '0.3'
        ])

    def alert_callback(self, msg):
        if msg.data and self.state not in ["RESCUE", "STANDBY"]:
            self.get_logger().info(f"[{self.name.upper()}] Broadcast received! Returning to Safe Zone.")
            self.state = "RESCUE"
            self.rescue_phase = 0

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

        if self.name == 'robot_1' and not self.victim_spawned and self.loop_ticks > 40:
            self.spawn_victim()
            self.victim_spawned = True

        dist_to_victim = math.hypot(self.current_x - self.victim_x, self.current_y - self.victim_y)
        victim_by_math = dist_to_victim < 2.0
        victim_by_contact = self.min_front < 1.0 and self.state in ["MARCH", "SWEEP_TURN_1", "SWEEP_SHIFT", "SWEEP_TURN_2"]

        if (victim_by_math or victim_by_contact) and self.state not in ["RESCUE", "STANDBY"]:
            self.get_logger().info(f"[{self.name.upper()}] 🚨 VICTIM LOCATED! Broadcasting to Swarm. 🚨")
            self.state = "RESCUE"
            self.rescue_phase = 0
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

            lane_center = (self.lane_min_x + self.lane_max_x) / 2.0
            reached_lane = self.name != 'robot_1' and abs(self.current_x - lane_center) < 0.3

            if reached_lane or self.shift_ticks > 200:
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

            hit_lane_edge = (self.room_shift_dir > 0 and self.current_x >= self.lane_max_x) or \
                            (self.room_shift_dir < 0 and self.current_x <= self.lane_min_x)

            if self.shift_ticks > 30 or self.min_front < 0.8 or hit_lane_edge:
                if self.min_front < 0.8 or hit_lane_edge:
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

        elif self.state == "STANDBY":
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif self.state == "RESCUE":
            tx, ty = self.rescue_waypoints[self.rescue_phase]
            dx, dy = tx - self.current_x, ty - self.current_y
            dist = math.hypot(dx, dy)

            if dist < 0.5:
                if self.rescue_phase < len(self.rescue_waypoints) - 1:
                    self.rescue_phase += 1
                    self.rescue_stuck_ticks = 0
                else:
                    self.state = "STANDBY"
                    self.get_logger().info(f"[{self.name.upper()}] ✅ Reached Safe Zone.")
            else:
                desired_yaw = math.atan2(dx, -dy)
                err = self.normalize_angle(desired_yaw - self.current_yaw)

                if self.min_front > 0.5:
                    cmd.angular.z = max(-1.2, min(1.2, 3.0 * err))
                    if abs(err) < 0.6:
                        cmd.linear.x = 0.5
                    self.rescue_stuck_ticks = 0
                else:
                    self.rescue_stuck_ticks += 1
                    cmd.linear.x = -0.3
                    cmd.angular.z = 0.0
                    if self.rescue_stuck_ticks > 40:
                        cmd.angular.z = 0.8

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MathematicalSwarm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
