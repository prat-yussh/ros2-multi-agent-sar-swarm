#!/usr/bin/env python3
"""
hospital_navigator.py  (v8 - centered waypoint routing, fixes wall-proximity stall)

CHANGE FROM v7: /odom logs showed robot1 completely frozen (position unchanging
across samples) while still 5m from its goal and in state EN_ROUTE - not an
oscillation, a genuine stall. Root cause: the final approach passed close
enough to the room's own side wall to enter the 1.5m repulsion radius. The
repulsive push away from the wall combined with the attractive pull toward
the goal landed in a local minimum - a classic failure mode of pure
potential-field navigation in narrow passages, not fixable by gain-tuning
alone.

Fix: the route is now THREE fixed legs per direction, each guaranteed by
construction to stay centered and away from any side wall:
  1. home -> hallway centerline waypoint directly outside the doorway
     (repulsion ON - this leg is shared hallway space, other robots may
     be present)
  2. waypoint -> a point just past the doorway threshold, still centered
     on the room's x (repulsion OFF - by construction this leg is centered
     in the doorway gap, no wall avoidance needed)
  3. threshold -> ward center (repulsion OFF - short final hop, fully
     inside the room)
Repulsion is only needed to avoid other robots in the shared hallway (leg 1);
it was only ever reacting to walls on legs 2-3, so removing it there
eliminates the local-minimum stall entirely.

ROS 2 Jazzy node implementing a home -> deliver -> return mission for one
differential-drive robot in the hospital_ward world.
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


REPULSE_RADIUS = 1.5
GOAL_TOLERANCE = 0.3
MAX_LINEAR_SPEED = 0.6
MAX_ANGULAR_SPEED = 1.5
ATTRACTIVE_GAIN = 1.0
REPULSIVE_GAIN = 1.8
DWELL_AT_HOME = 3.0
DWELL_AT_WARD = 4.0

ROTATE_ENTER_THRESHOLD = 0.6
ROTATE_EXIT_THRESHOLD = 0.15

STATE_AT_HOME = 'AT_HOME'
STATE_EN_ROUTE_TO_WAYPOINT = 'EN_ROUTE_TO_WAYPOINT'
STATE_EN_ROUTE_TO_THRESHOLD = 'EN_ROUTE_TO_THRESHOLD'
STATE_EN_ROUTE = 'EN_ROUTE'
STATE_DELIVERING = 'DELIVERING'
STATE_RETURNING_TO_THRESHOLD = 'RETURNING_TO_THRESHOLD'
STATE_RETURNING_TO_WAYPOINT = 'RETURNING_TO_WAYPOINT'
STATE_RETURNING = 'RETURNING'
STATE_MISSION_COMPLETE = 'MISSION_COMPLETE'


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HospitalNavigator(Node):
    def __init__(self, robot_name: str, home_x: float, home_y: float,
                 ward_x: float, ward_y: float, ward_name: str, loop: bool):
        super().__init__(f'{robot_name}_navigator')

        self.robot_name = robot_name
        self.home_x = home_x
        self.home_y = home_y
        self.ward_x = ward_x
        self.ward_y = ward_y
        self.ward_name = ward_name
        self.loop = loop

        self.waypoint_x = ward_x
        self.waypoint_y = 0.0

        doorway_y = 2.25 if ward_y > 0 else -2.25
        direction = 1.0 if ward_y > 0 else -1.0
        self.threshold_x = ward_x
        self.threshold_y = doorway_y + direction * 1.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False
        self.latest_scan = None

        self.state = STATE_AT_HOME
        self.state_entry_time = None
        self.rotating_in_place = False

        self.cmd_pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, f'/{robot_name}/mission_status', 10)

        self.odom_sub = self.create_subscription(
            Odometry, f'/{robot_name}/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{robot_name}/scan', self.scan_callback,
            qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'{robot_name} navigator started. Home: ({home_x:.2f}, {home_y:.2f}), '
            f'Waypoint: ({self.waypoint_x:.2f}, {self.waypoint_y:.2f}), '
            f'Threshold: ({self.threshold_x:.2f}, {self.threshold_y:.2f}), '
            f'Ward: {ward_name} ({ward_x:.2f}, {ward_y:.2f})')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x + self.home_x
        self.current_y = msg.pose.pose.position.y + self.home_y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if not self.have_pose:
            self.state_entry_time = self.get_clock().now()
        self.have_pose = True

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def compute_repulsive_vector(self):
        rx, ry = 0.0, 0.0
        if self.latest_scan is None:
            return rx, ry
        scan = self.latest_scan
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < REPULSE_RADIUS:
                strength = (REPULSE_RADIUS - r) / REPULSE_RADIUS
                rx += -math.cos(angle) * strength
                ry += -math.sin(angle) * strength
            angle += scan.angle_increment
        return rx, ry

    def seconds_in_state(self) -> float:
        if self.state_entry_time is None:
            return 0.0
        return (self.get_clock().now() - self.state_entry_time).nanoseconds / 1e9

    def enter_state(self, new_state: str):
        self.state = new_state
        self.state_entry_time = self.get_clock().now()
        self.rotating_in_place = False
        msg = String()
        msg.data = f'{new_state}'
        self.status_pub.publish(msg)
        self.get_logger().info(f'{self.robot_name}: -> {new_state}')

    def drive_toward(self, target_x: float, target_y: float, use_repulsion: bool) -> float:
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.hypot(dx, dy)

        if distance < GOAL_TOLERANCE:
            self.cmd_pub.publish(Twist())
            return distance

        att_x = (dx / distance) * ATTRACTIVE_GAIN
        att_y = (dy / distance) * ATTRACTIVE_GAIN

        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        att_body_x = att_x * cos_yaw + att_y * sin_yaw
        att_body_y = -att_x * sin_yaw + att_y * cos_yaw

        attractive_heading_error = math.atan2(att_body_y, att_body_x)

        if use_repulsion:
            rep_x, rep_y = self.compute_repulsive_vector()
            rep_x *= REPULSIVE_GAIN
            rep_y *= REPULSIVE_GAIN
            total_x = att_body_x + rep_x
            total_y = att_body_y + rep_y
            combined_heading_error = math.atan2(total_y, total_x)
        else:
            combined_heading_error = attractive_heading_error

        twist = Twist()

        if self.rotating_in_place:
            if abs(attractive_heading_error) < ROTATE_EXIT_THRESHOLD:
                self.rotating_in_place = False
        else:
            if abs(attractive_heading_error) > ROTATE_ENTER_THRESHOLD:
                self.rotating_in_place = True

        if self.rotating_in_place:
            twist.linear.x = 0.0
            twist.angular.z = max(-MAX_ANGULAR_SPEED,
                                   min(MAX_ANGULAR_SPEED, 1.5 * attractive_heading_error))
        else:
            turn_factor = max(0.0, 1.0 - abs(combined_heading_error) / math.pi)
            twist.linear.x = min(MAX_LINEAR_SPEED, MAX_LINEAR_SPEED * turn_factor + 0.05)
            twist.angular.z = max(-MAX_ANGULAR_SPEED,
                                   min(MAX_ANGULAR_SPEED, 2.0 * combined_heading_error))

        self.cmd_pub.publish(twist)
        return distance

    def control_loop(self):
        if not self.have_pose:
            return

        if self.state == STATE_AT_HOME:
            self.cmd_pub.publish(Twist())
            if self.seconds_in_state() >= DWELL_AT_HOME:
                self.enter_state(STATE_EN_ROUTE_TO_WAYPOINT)

        elif self.state == STATE_EN_ROUTE_TO_WAYPOINT:
            distance = self.drive_toward(self.waypoint_x, self.waypoint_y, use_repulsion=True)
            if distance < GOAL_TOLERANCE:
                self.enter_state(STATE_EN_ROUTE_TO_THRESHOLD)

        elif self.state == STATE_EN_ROUTE_TO_THRESHOLD:
            distance = self.drive_toward(self.threshold_x, self.threshold_y, use_repulsion=False)
            if distance < GOAL_TOLERANCE:
                self.enter_state(STATE_EN_ROUTE)

        elif self.state == STATE_EN_ROUTE:
            distance = self.drive_toward(self.ward_x, self.ward_y, use_repulsion=False)
            if distance < GOAL_TOLERANCE:
                self.enter_state(STATE_DELIVERING)

        elif self.state == STATE_DELIVERING:
            self.cmd_pub.publish(Twist())
            if self.seconds_in_state() >= DWELL_AT_WARD:
                self.enter_state(STATE_RETURNING_TO_THRESHOLD)

        elif self.state == STATE_RETURNING_TO_THRESHOLD:
            distance = self.drive_toward(self.threshold_x, self.threshold_y, use_repulsion=False)
            if distance < GOAL_TOLERANCE:
                self.enter_state(STATE_RETURNING_TO_WAYPOINT)

        elif self.state == STATE_RETURNING_TO_WAYPOINT:
            distance = self.drive_toward(self.waypoint_x, self.waypoint_y, use_repulsion=False)
            if distance < GOAL_TOLERANCE:
                self.enter_state(STATE_RETURNING)

        elif self.state == STATE_RETURNING:
            distance = self.drive_toward(self.home_x, self.home_y, use_repulsion=True)
            if distance < GOAL_TOLERANCE:
                if self.loop:
                    self.enter_state(STATE_AT_HOME)
                else:
                    self.enter_state(STATE_MISSION_COMPLETE)

        elif self.state == STATE_MISSION_COMPLETE:
            self.cmd_pub.publish(Twist())


def main():
    parser = argparse.ArgumentParser(description='Hospital swarm navigator')
    parser.add_argument('--robot_name', type=str, required=True)
    parser.add_argument('--home_x', type=float, required=True)
    parser.add_argument('--home_y', type=float, required=True)
    parser.add_argument('--ward_x', type=float, required=True)
    parser.add_argument('--ward_y', type=float, required=True)
    parser.add_argument('--ward_name', type=str, default='Ward')
    parser.add_argument('--loop', action='store_true')
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = HospitalNavigator(parsed.robot_name, parsed.home_x, parsed.home_y,
                              parsed.ward_x, parsed.ward_y, parsed.ward_name,
                              parsed.loop)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
