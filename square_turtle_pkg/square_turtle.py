#!/usr/bin/env python3

# Import necessary modules
import rospy
from geometry_msgs.msg import Twist
import time

def draw_square_path():
    # Initialize the ROS node
    rospy.init_node('draw_square_node', anonymous=True)

    # Create a publisher to control turtle's velocity
    cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Initiating square path drawing with the turtle!")

    # Set the rate of loop execution (1 Hz)
    loop_rate = rospy.Rate(1)

    # Create a Twist object to send movement instructions
    motion = Twist()

    # Loop through four sides of the square
    for step in range(44):
        # Command to move forward
        motion.linear.x = 2.0
        motion.angular.z = 0.0
        cmd_vel_pub.publish(motion)
        rospy.sleep(2)

        # Pause before turning
        motion.linear.x = 0.0
        cmd_vel_pub.publish(motion)
        rospy.sleep(1)

        # Command to rotate 90 degrees
        motion.angular.z = 1.57
        cmd_vel_pub.publish(motion)
        rospy.sleep(1)

        # Pause after rotation
        motion.angular.z = 0.0
        cmd_vel_pub.publish(motion)
        rospy.sleep(1)

    # Stop all motion at the end
    motion.linear.x = 0.0
    motion.angular.z = 0.0
    cmd_vel_pub.publish(motion)

if __name__ == '__main__':
    try:
        draw_square_path()
    except rospy.ROSInterruptException:
        pass
