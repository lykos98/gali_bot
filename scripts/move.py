#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def now():
    return rospy.get_time()


def move():
    # Initialize the node
    rospy.init_node('robot_mover', anonymous=True)
    
    # Create a publisher to the /cmd_vel topic
    # Twist is the message type (Linear and Angular velocity)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    
    # Define the movement
    move_straight = Twist()
    move_straight.linear.x = 0.2  # Move forward at 0.2 m/s
    move_straight.angular.z = 0.0 # Don't turn

    rate = rospy.Rate(20)
    
    turn_right = Twist()
    turn_right.linear.x = 0.0  # Stop moving 
    turn_right.angular.z = 0.1 # Turn

    # Loop until you press Ctrl+C

    while not rospy.is_shutdown():
        start = now()
        while now() - start < 3:
            print(now() - start)
            pub.publish(move_straight)
            rate.sleep()

        start = now()
        while now() - start < 2:
            print(now() - start)
            pub.publish(turn_right)
            rate.sleep()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass
