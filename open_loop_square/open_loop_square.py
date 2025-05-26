#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class Drive_Square:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()

        rospy.init_node('drive_square_node', anonymous=True)

        self.pub = rospy.Publisher('/jaatt/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/jaatt/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

        self.has_moved = False

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING" and not self.has_moved:
            rospy.sleep(1)
            self.move_robot()
            self.has_moved = True

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def move_forward(self, speed, duration):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.sleep(duration)
        self.stop_robot()
        rospy.sleep(0.5)

    def turn_90_degrees(self, angular_speed, duration):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = angular_speed
        self.pub.publish(self.cmd_msg)
        rospy.sleep(duration)
        self.stop_robot()
        rospy.sleep(0.5)

    def move_robot(self):
        forward_speed = 0.3
        forward_time = 3.3  # ~1 meter at 0.3 m/s

        angular_speed = 1.0
        turn_time = 1.57  # ~90 degrees at 1.0 rad/s

        for i in range(4):
            rospy.loginfo(f"Side {i+1}: Moving forward")
            self.move_forward(forward_speed, forward_time)

            rospy.loginfo(f"Corner {i+1}: Turning 90 degrees")
            self.turn_90_degrees(angular_speed, turn_time)

        rospy.loginfo("Finished square")
        self.stop_robot()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        duckiebot_movement = Drive_Square()
        duckiebot_movement.run()
    except rospy.ROSInterruptException:
        pass
