import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    # The 'ranges' list has 360 values (one for each degree)
    # index 0 is directly in front of the robot
    front_distance = msg.ranges[0]
    rospy.loginfo(f"Distance in front: {front_distance:.2f} meters")

def listener():
    rospy.init_node('lidar_reader', anonymous=True)
    
    # Subscribe to the 'scan' topic
    rospy.Subscriber("scan", LaserScan, callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    listener()
