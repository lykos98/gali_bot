import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist




def now():
    return rospy.get_time()

class ObstacleAvoider:
    def __init__(self):
        self.distances = None
        rospy.init_node('robot_mover', anonymous=True)
        self.fov = 15
        self.min_dist = 0.60
        self.cone_front = 0
        self.cone_left = 0
        self.cone_right = 0

        # Subscribe to the 'scan' topic
        rospy.Subscriber("scan", LaserScan, self.__callback)

    def __callback(self,msg):
        self.distances = np.array(msg.ranges)
        ww = np.where(self.distances < 1e-3)
        self.distances[ww] = np.inf
        self.__compute_cones()
        

    def __compute_cones(self):
        
        #self.cone_right = self.distances[self.fov:2*self.fov].mean()
        #self.cone_left = self.distances[-2*self.fov:-self.fov].mean()
        #self.cone_front = (self.distances[-self.fov:].sum() + self.distances[:self.fov].sum())/(2*self.fov)
        
        self.cone_right = self.distances[self.fov:3*self.fov].min()
        self.cone_left = self.distances[-3*self.fov:-self.fov].min()
        self.cone_front = min(self.distances[-self.fov:].min(), self.distances[:self.fov].min())


    def run(self):
        # Initialize the node
        
        # Create a publisher to the /cmd_vel topic
        # Twist is the message type (Linear and Angular velocity)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        
        # Define the movement
        move_straight = Twist()
        move_straight.linear.x = 0.4  # Move forward at 0.2 m/s
        move_straight.angular.z = 0.0 # Don't turn

        rate = rospy.Rate(150)
        
        turn_right = Twist()
        turn_right.linear.x = 0.1  # Stop moving 
        turn_right.angular.z = 0.8 # Turn

        turn_left = Twist()
        turn_left.linear.x = 0.1 # Stop moving 
        turn_left.angular.z = -0.8 # Turn

        stop = Twist()
        stop.linear.x = .0
        stop.angular.z = .0

        # Loop until you press Ctrl+C

        while not rospy.is_shutdown():
            front = self.cone_front
            left = self.cone_left
            right = self.cone_right
            
            if front < self.min_dist:
                # chose direction 
                #pub.publish(stop)
                start = now()
                if right > left:
                    # go right
                    rospy.loginfo("Taking a turn right")
                    while (now() - start) < 2.:
                        pub.publish(turn_right)
                        rate.sleep()
                else:
                    # go left
                    rospy.loginfo("Taking a turn left")
                    while (now() - start) < 2.:
                        pub.publish(turn_left)
                        rate.sleep()

                #pub.publish(stop)
            pub.publish(move_straight)

            rate.sleep()

if __name__ == '__main__':
    mover = ObstacleAvoider()
    mover.run()
