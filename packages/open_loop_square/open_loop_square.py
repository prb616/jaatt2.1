#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState

class ForwardBackwardMover:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        rospy.init_node('forward_backward_node', anonymous=True)
        self.pub = rospy.Publisher('/jaatt/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        rospy.Subscriber('/jaatt/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
        elif msg.state == "LANE_FOLLOWING":
            rospy.sleep(1)
            self.forward_backward_sequence()

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo("Robot Stopped")

    def forward_backward_sequence(self):
        self.move_forward(1.6, 1.5)    # Move forward at 0.6 m/s for 1.5 sec
        self.stop_robot()
        rospy.sleep(1)
        self.move_backward(1.0, 1.5)   # Move backward at 0.3 m/s for 1.5 sec
        self.stop_robot()

    def move_forward(self, speed, duration):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving forward at {speed} m/s...")
        rospy.sleep(duration)

    def move_backward(self, speed, duration):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = -speed
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)
        rospy.loginfo(f"Moving backward at {speed} m/s...")
        rospy.sleep(duration)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mover = ForwardBackwardMover()
        mover.run()
    except rospy.ROSInterruptException:
        pass
