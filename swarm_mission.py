import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random

class SwarmMission(Node):
    def __init__(self):
        super().__init__('swarm_mission')
        
        # State: 0=STANDBY, 1=SEARCHING, 2=RESCUING
        self.state = 0
        self.timer_count = 0
        
        # Safe Zone coordinates
        self.safe_zone_x = 0.0
        self.safe_zone_y = 12.0
        
        # Central Doorway Gap (to bypass corridor_left and corridor_right walls)
        self.doorway_x = 0.0
        self.doorway_y = 5.0
        
        # Victim location
        self.victim_x = 0.0
        self.victim_y = 0.0
        
        # Robot data storage
        self.robots = {
            1: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'front': 10.0, 'left': 10.0, 'right': 10.0},
            2: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'front': 10.0, 'left': 10.0, 'right': 10.0},
            3: {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'front': 10.0, 'left': 10.0, 'right': 10.0}
        }
        
        # Publishers and Subscribers
        self.pubs = {}
        for i in range(1, 4):
            self.pubs[i] = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/robot_{i}/odom', lambda msg, id=i: self.odom_cb(msg, id), 10)
            self.create_subscription(LaserScan, f'/robot_{i}/scan', lambda msg, id=i: self.laser_cb(msg, id), 10)

        self.timer = self.create_timer(0.1, self.mission_loop)
        self.get_logger().info("=== SWARM DEPLOYED: STANDBY MODE ===")

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_cb(self, msg, robot_id):
        self.robots[robot_id]['x'] = msg.pose.pose.position.x
        self.robots[robot_id]['y'] = msg.pose.pose.position.y
        self.robots[robot_id]['yaw'] = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def laser_cb(self, msg, robot_id):
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return
        
        cleaned = [r if (0.1 < r < 30.0) else 10.0 for r in ranges]
        
        # 3-Sector Laser Vision
        right_sector = cleaned[0 : int(n * 0.35)]
        front_sector = cleaned[int(n * 0.35) : int(n * 0.65)]
        left_sector  = cleaned[int(n * 0.65) : n]

        self.robots[robot_id]['front'] = min(front_sector) if front_sector else 10.0
        self.robots[robot_id]['left']  = min(left_sector)  if left_sector  else 10.0
        self.robots[robot_id]['right'] = min(right_sector) if right_sector else 10.0

    def move_robot(self, robot_id, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pubs[robot_id].publish(msg)

    def mission_loop(self):
        self.timer_count += 1
        
        # PHASE 1 & 2: STANDBY AND ALARM
        if self.state == 0:
            if self.timer_count == 50: # 5 Seconds
                self.victim_x = random.uniform(-8.0, 8.0)
                self.victim_y = random.uniform(-5.0, 2.0)
                self.get_logger().info(f"!!! ALARM TRIGGERED !!! Victim reported near X:{self.victim_x:.1f}, Y:{self.victim_y:.1f}")
                self.get_logger().info("SWARM: Initiating Search Protocol!")
                self.state = 1
            return

        # PHASE 3: SEARCHING
        if self.state == 1:
            for i in range(1, 4):
                rx = self.robots[i]['x']
                ry = self.robots[i]['y']
                dist_to_victim = math.sqrt((rx - self.victim_x)**2 + (ry - self.victim_y)**2)
                
                if dist_to_victim < 2.0:
                    self.get_logger().info(f">>> ROBOT {i} FOUND THE VICTIM! Broadcasting to team! <<<")
                    self.get_logger().info("SWARM: Escorting victim directly through central doorway to Safe Zone!")
                    self.state = 2
                    return
                
                # Search movement with smart wall avoidance
                f_dist = self.robots[i]['front']
                l_dist = self.robots[i]['left']
                r_dist = self.robots[i]['right']

                if f_dist < 1.0:
                    turn = 0.6 if l_dist > r_dist else -0.6
                    self.move_robot(i, 0.0, turn)
                else:
                    self.move_robot(i, 0.35, 0.0)

        # PHASE 4 & 5: RESCUE VIA DOORWAY WAYPOINT
        if self.state == 2:
            for i in range(1, 4):
                rx = self.robots[i]['x']
                ry = self.robots[i]['y']
                ryaw = self.robots[i]['yaw']
                
                f_dist = self.robots[i]['front']
                l_dist = self.robots[i]['left']
                r_dist = self.robots[i]['right']

                # Waypoint Logic: Pass central doorway (0, 5) first if south of the wall
                if ry < 4.8:
                    target_x, target_y = self.doorway_x, self.doorway_y
                else:
                    target_x, target_y = self.safe_zone_x, self.safe_zone_y

                # Obstacle Avoidance Override
                if f_dist < 0.9:
                    turn = 0.7 if l_dist > r_dist else -0.7
                    self.move_robot(i, 0.0, turn)
                else:
                    target_angle = math.atan2(target_y - ry, target_x - rx)
                    angle_diff = target_angle - ryaw
                    
                    while angle_diff > math.pi: angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi: angle_diff += 2 * math.pi
                    
                    self.move_robot(i, 0.35, max(-0.8, min(0.8, angle_diff * 1.5)))

def main(args=None):
    rclpy.init(args=args)
    node = SwarmMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
