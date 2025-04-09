#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        self.pose = None
        self.start_pose = None

        self.goal_distance = 0.0
        self.distance_active = False
        self.forward = True

        self.goal_angle = 0.0
        self.angle_active = False
        self.rotate_left = True
        self.start_angle = 0.0

        self.goal_position = None
        self.position_active = False

        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        rospy.Subscriber("/turtle1/pose", Pose, self.pose_callback)
        rospy.Subscriber("/goal_distance", Float64, self.goal_distance_callback)
        rospy.Subscriber("/goal_angle", Float64, self.goal_angle_callback)
        rospy.Subscriber("/goal_position", Point, self.goal_position_callback)

        self.vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.01), self.timer_callback)
        rospy.loginfo("Turtle command node started")
        rospy.spin()

    def pose_callback(self, msg):
        self.pose = msg

    def goal_distance_callback(self, msg):
        if not self.pose:
            return
        if msg.data == 0.0:
            return
        self.goal_distance = abs(msg.data)
        self.forward = msg.data > 0
        self.distance_active = True
        self.start_pose = self.pose
        rospy.loginfo("Distance goal received: %f", msg.data)

    def goal_angle_callback(self, msg):
        if not self.pose:
            return
        if msg.data == 0.0:
            return
        self.goal_angle = abs(msg.data)
        self.rotate_left = msg.data > 0
        self.angle_active = True
        self.start_angle = self.pose.theta
        rospy.loginfo("Angle goal received: %f", msg.data)

    def goal_position_callback(self, msg):
        if not self.pose:
            return
        self.goal_position = msg
        self.position_active = True
        rospy.loginfo("Position goal received: (%.2f, %.2f)", msg.x, msg.y)

    def compute_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def angle_diff(self, a, b):
        diff = a - b
        return math.atan2(math.sin(diff), math.cos(diff))

    def timer_callback(self, event):
        if not self.pose:
            return

        cmd = Twist()

        # 1. Handle distance movement
        if self.distance_active:
            moved = self.compute_distance(self.pose, self.start_pose)
            if moved >= self.goal_distance:
                self.distance_active = False
                rospy.loginfo("Reached distance goal")
            else:
                cmd.linear.x = 2.0 if self.forward else -2.0
                self.vel_pub.publish(cmd)
                return

        # 2. Handle rotation
        if self.angle_active:
            delta = self.angle_diff(self.pose.theta, self.start_angle)
            if abs(delta) >= self.goal_angle:
                self.angle_active = False
                rospy.loginfo("Reached angle goal")
            else:
                cmd.angular.z = 1.5 if self.rotate_left else -1.5
                self.vel_pub.publish(cmd)
                return

        # 3. Handle position goal
        if self.position_active and self.goal_position:
            dx = self.goal_position.x - self.pose.x
            dy = self.goal_position.y - self.pose.y
            target_angle = math.atan2(dy, dx)
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_turn = self.angle_diff(target_angle, self.pose.theta)

            if abs(angle_to_turn) > 0.1:
                cmd.angular.z = 1.5 if angle_to_turn > 0 else -1.5
            elif distance > 0.2:
                cmd.linear.x = 2.0
            else:
                self.position_active = False
                rospy.loginfo("Reached position goal")

            self.vel_pub.publish(cmd)
            return

        # 4. Stop if no goal
        self.vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException:
        pass
