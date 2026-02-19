This lesson transitions from manual control to **automated sensing and movement**. It’s the bridge between "driving a remote control car" and "programming a robot."

Here is the raw Markdown for **Lesson 2**.

---

# Lesson 2: Sensing and Autonomous Movement

## 1. Objectives

* Visualize sensor data in **RViz**.
* Understand the difference between **Gazebo** (the world) and **RViz** (the robot's mind).
* Write and run a **Python script** to control the robot.

---

## 2. Part A: Seeing through the Robot's Eyes (RViz)

While Gazebo shows us the "truth" (what the world looks like), **RViz (ROS Visualization)** shows us what the **robot perceives**.

### Step 1: Launch RViz

In a new terminal:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_remote.launch
rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/model.rviz

```

### Step 2: Key Displays in RViz

* **Robot Model:** A 3D representation of the Burger bot.
* **LaserScan:** You should see small **Red Dots**. These represent the "hits" from the LiDAR.
* **Global Options -> Fixed Frame:** This should usually be set to `odom` or `map`. If set incorrectly, the sensor data will look broken.

> **Exercise:** In Gazebo, place a cube in front of the robot. Watch how a line of red dots appears in RViz. This is how the robot "sees" a wall.

---

## 3. Part B: Commanding the Robot via Python

Instead of using the keyboard, we will write a script to send `Twist` messages.

### Step 1: Create the Script

Create a file named `simple_move.py` in your workspace:

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # 1. Initialize the ROS Node
    rospy.init_node('robot_cleaner', anonymous=True)
    
    # 2. Create a Publisher to /cmd_vel
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 3. Define the speed (0.2 m/s forward)
    vel_msg = Twist()
    vel_msg.linear.x = 0.2
    vel_msg.angular.z = 0.0

    # 4. Loop at 10Hz
    rate = rospy.Rate(10) 
    
    rospy.loginfo("Moving forward...")
    
    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

```

### Step 2: Make it Executable

In the terminal, you must give your computer permission to run the script:

```bash
chmod +x simple_move.py
python3 simple_move.py

```

---

## 4. Part C: The "Physics" of Movement (Twist)

The `Twist` message is part of the `geometry_msgs` package. It contains two main vectors: **Linear** and **Angular**.

| Attribute | Axis | Direction |
| --- | --- | --- |
| `linear.x` | X-axis | Forward (+) / Backward (-) |
| `angular.z` | Z-axis | Turn Left (+) / Turn Right (-) |

---

## 5. Lab Exercises

### Exercise 1: The Circular Path

Modify your `simple_move.py` script so that the robot drives in a circle with a radius of 0.5 meters.

* *Hint:* Set `linear.x = 0.2`. Use the formula  to calculate the required `angular.z`.

### Exercise 2: The "Safety Stop" (Theory)

Look at the `LaserScan` message using `rostopic echo /scan -n 1`.

1. Identify the `ranges` field.
2. **Question:** If the robot is driving forward, which index in the `ranges` array corresponds to the "Dead Ahead" direction? (Index 0, 90, 180, or 270?)

### Exercise 3: RViz Customization

In RViz, change the **Size** of the LaserScan points (from 0.01 to 0.05) and change the **Color Transformer** to "AxisColor".

* **Observation:** What happens to the color of the dots as you rotate the robot toward a wall?

---

### 🎓 Instructor Notes

* **RViz vs Gazebo:** Explain that Gazebo is the "Simulator Engine" while RViz is the "Developer's Dashboard." You can run RViz on a real robot, but you only run Gazebo for simulations.
* **The Rate Object:** Explain that `rospy.Rate(10)` ensures the loop runs exactly 10 times per second. Without this, the script would consume 100% of the CPU.

**Would you like me to create the Python template for Lesson 3, where we combine the Lidar data with the movement logic?**