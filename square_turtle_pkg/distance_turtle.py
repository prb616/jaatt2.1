#!/usr/bin/env python3

# Import required libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import math

class DistanceReader:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlesim_distance_node', anonymous=True)

        # Set up subscriber to listen to turtle's pose
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)

        # Set up publisher to send total distance
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)

        # Variables to track previous position and calculate distance
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0

        rospy.loginfo("Distance tracking node has been initialized.")

        # Keep the node running
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("Turtle's Current Position: x=%.2f, y=%.2f", msg.x, msg.y)

        if self.prev_x is not None and self.prev_y is not None:
            # Compute the distance moved since the last position update
            dx = msg.x - self.prev_x
            dy = msg.y - self.prev_y
            distance = math.sqrt(dx*2 + dy*2)

            # Add this distance to the overall total distance
            self.total_distance += distance

            # Publish the updated total distance
            self.distance_publisher.publish(self.total_distance)

            rospy.loginfo("Cumulative Distance Travelled: %.2f", self.total_distance)

        # Update previous coordinates for future distance calculations
        self.prev_x = msg.x
        self.prev_y = msg.y

# Check if the script is being executed directly and not imported as a module
if __name__ == '__main__':
    try:
        DistanceReader()
    except rospy.ROSInterruptException:
        pass
