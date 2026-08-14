import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random
import os

class ConvoySwarmRescue(Node):
    def __init__(self):
        super().__init__('convoy_swarm_rescue')
        
        self.state = 'STANDBY'
        self.timer_count = 0
        self.load_timer = 0

        # Victim Zone (Right side of the wall)
        self.victim_x = random.uniform(3.0, 6.0)
        self.victim_y = random.uniform(1.0, 4.0)
        
        self.get_logger().info("Spawning 3D Physical Victim...")
        spawn_cmd = f"ros2 run ros_gz_sim create -world disaster_hospital -name physical_victim -string \"<sdf version='1.6'><model name='victim'><static>true</static><link name='link'><visual name='visual'><geometry><box><size>0.6 0.6 0.6</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>\" -x {self.victim_x} -y {self.victim_y} -z 0.3 &"
        os.system(spawn_cmd)
        
        # Single-File Staging Coordinates (Lining up one behind the other)
        self.stage_in = {1: (-1.5, 5.0), 2: (-1.5, 4.0), 3: (-1.5, 3.0)}
        self.stage_out = {1: (1.5, 5.0), 2: (1.5, 4.0), 3: (1.5, 3.0)}
        
        # Cross coordinates
        self.cross_door_right = (2.0, 5.0)
        self.cross_door_left = (-2.0, 5.0)

        # Parking coordinates for Green Mat
        self.safe_zone = {1: (-7.0, -2.0), 2: (-8.0, -2.0), 3: (-6.0, -2.0)}

        self.search_targets = {
            1: (self.victim_x, self.victim_y),
            2: (self.victim_x + 1.2, self.victim_y + 1.2),
            3: (self.victim_x + 1.2, self.victim_y - 1.2)
        }

        self.offsets = {}
        self.poses = {1: [0.0, 0.0, 0.0], 2: [0.0, 0.0, 0.0], 3: [0.0, 0.0, 0.0]}
        self.finder_robot = 1

        self.pubs = {}
        for i in range(1, 4):
            self.pubs[i] = self.create_publisher(Twist, f'/robot_{i}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/robot_{i}/odom', lambda msg, id=i: self.odom_cb(msg, id), 10)
        
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("=== CONVOY SWARM SYSTEM READY ===")

    def odom_cb(self, msg, r_id):
        if r_id not in self.offsets:
            spawn_x = -8.0 if r_id == 1 else -9.0
            spawn_y = -2.0 if r_id == 1 else (-0.5 if r_id == 2 else -3.5)
            self.offsets[r_id] = (spawn_x - msg.pose.pose.position.x, spawn_y - msg.pose.pose.position.y)
            
        ox, oy = self.offsets[r_id]
        self.poses[r_id][0] = msg.pose.pose.position.x + ox
        self.poses[r_id][1] = msg.pose.pose.position.y + oy
        
        q = msg.pose.pose.orientation
        self.poses[r_id][2] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def move_robot(self, r_id, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pubs[r_id].publish(msg)

    def drive_tank(self, r_id, tx, ty, tolerance=0.5):
        rx, ry, ryaw = self.poses[r_id]
        if math.hypot(tx - rx, ty - ry) < tolerance:
            self.move_robot(r_id, 0.0, 0.0)
            return True

        target_angle = math.atan2(ty - ry, tx - rx)
        diff = target_angle - ryaw
        
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        
        if abs(diff) > 0.15: 
            self.move_robot(r_id, 0.0, max(-1.0, min(1.0, diff * 2.5)))
        else:
            self.move_robot(r_id, 0.65, 0.0) 
        return False

    def loop(self):
        self.timer_count += 1
        
        if self.state == 'STANDBY':
            if self.timer_count == 40:
                self.get_logger().info("!!! ALARM !!! Lining up at doorway in single-file convoy...")
                self.state = 'STAGE_LEFT'
            return

        # PHASE 1: Form a single-file line facing the door
        elif self.state == 'STAGE_LEFT':
            all_arrived = True
            for i in range(1, 4):
                if not self.drive_tank(i, self.stage_in[i][0], self.stage_in[i][1], tolerance=0.5):
                    all_arrived = False
            if all_arrived:
                self.get_logger().info("SWARM: Convoy ready. Proceeding through gap...")
                self.state = 'CROSS_RIGHT'

        # PHASE 2: Drive straight through the door
        elif self.state == 'CROSS_RIGHT':
            all_cleared = True
            for i in range(1, 4):
                if not self.drive_tank(i, self.cross_door_right[0], self.cross_door_right[1], tolerance=0.8):
                    all_cleared = False
            if all_cleared:
                self.get_logger().info("SWARM: Wall cleared! Spreading out to search.")
                self.state = 'SEARCH'

        # PHASE 3: Fan out and search
        elif self.state == 'SEARCH':
            for i in range(1, 4):
                if math.hypot(self.poses[i][0] - self.victim_x, self.poses[i][1] - self.victim_y) < 1.0:
                    self.finder_robot = i
                    self.get_logger().info(f">>> ROBOT {i} SECURED THE VICTIM! Regrouping... <<<")
                    self.state = 'REGROUP'
                    return
            
            for i in range(1, 4):
                self.drive_tank(i, self.search_targets[i][0], self.search_targets[i][1])

        # PHASE 4: Surround victim
        elif self.state == 'REGROUP':
            self.move_robot(self.finder_robot, 0.0, 0.0) 
            all_arrived = True
            for i in range(1, 4):
                if i != self.finder_robot:
                    if not self.drive_tank(i, self.poses[self.finder_robot][0], self.poses[self.finder_robot][1], tolerance=1.3):
                        all_arrived = False
            
            if all_arrived:
                self.get_logger().info(">>> ASSEMBLED! LOADING VICTIM INTO CARGO... <<<")
                self.state = 'LOADING'
                self.load_timer = 0

        # PHASE 5: Despawn box
        elif self.state == 'LOADING':
            for i in range(1, 4): self.move_robot(i, 0.0, 0.0)
            self.load_timer += 1
            if self.load_timer > 30: 
                os.system("gz service -s /world/disaster_hospital/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 --req 'name: \"physical_victim\", type: MODEL'")
                self.get_logger().info(">>> VICTIM SECURED! Re-forming convoy at gap... <<<")
                self.state = 'STAGE_RIGHT'

        # PHASE 6: Form single-file line on the right side of the door
        elif self.state == 'STAGE_RIGHT':
            all_arrived = True
            for i in range(1, 4):
                if not self.drive_tank(i, self.stage_out[i][0], self.stage_out[i][1], tolerance=0.5):
                    all_arrived = False
            if all_arrived:
                self.state = 'CROSS_LEFT'

        # PHASE 7: Drive straight through the door back to left side
        elif self.state == 'CROSS_LEFT':
            all_cleared = True
            for i in range(1, 4):
                if not self.drive_tank(i, self.cross_door_left[0], self.cross_door_left[1], tolerance=0.8):
                    all_cleared = False
            if all_cleared:
                self.get_logger().info(">>> CLEARED WALL! Returning to Green Mat... <<<")
                self.state = 'SAFE_ZONE'

        # PHASE 8: Park on Green Mat
        elif self.state == 'SAFE_ZONE':
            all_parked = True
            for i in range(1, 4):
                if not self.drive_tank(i, self.safe_zone[i][0], self.safe_zone[i][1], tolerance=0.5):
                    all_parked = False
            if all_parked:
                self.get_logger().info(">>> REACHED GREEN MAT! UNLOADING... <<<")
                self.state = 'UNLOADING'
                self.load_timer = 0

        # PHASE 9: Respawn box as green
        elif self.state == 'UNLOADING':
            for i in range(1, 4): self.move_robot(i, 0.0, 0.0)
            self.load_timer += 1
            if self.load_timer > 30: 
                spawn_cmd = f"ros2 run ros_gz_sim create -world disaster_hospital -name rescued_victim -string \"<sdf version='1.6'><model name='victim'><static>true</static><link name='link'><visual name='visual'><geometry><box><size>0.6 0.6 0.6</size></box></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material></visual></link></model></sdf>\" -x -7.0 -y -2.0 -z 0.3 &"
                os.system(spawn_cmd)
                self.get_logger().info("!!! MISSION COMPLETELY ACCOMPLISHED !!!")
                self.state = 'MISSION_COMPLETE'

def main():
    rclpy.init()
    node = ConvoySwarmRescue()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
