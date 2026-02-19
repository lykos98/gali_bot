This is the "Grand Finale" of the introductory series. This lesson combines everything: **Subscribers** (Lidar), **Publishers** (Movement), and **Logic** (Obstacle Avoidance).

Below is the raw Markdown for **Lesson 3**.

---

# Lesson 3: Building an Autonomous Obstacle Avoider

## 1. Objectives

* Create a **Subscriber** to read Lidar data within a Python script.
* Implement a **Reactive Logic** loop (Sense  Think  Act).
* Use **NumPy** to process sensor arrays efficiently.

---

## 2. Part A: Understanding the Control Loop

To avoid obstacles, the robot must follow a continuous cycle:

1. **Sense:** Listen to the `/scan` topic.
2. **Think:** Look at the `ranges` array. Is the minimum distance in front < 0.5m?
3. **Act:** Send a "Stop" or "Turn" command to `/cmd_vel`.

---

## 3. Part B: The Python Logic (State Machine)

Instead of a simple "move" script, we will use a **Class-based** approach. This allows the Lidar callback to update the robot's "vision" while the main loop handles the "driving."

### Step 1: Create `avoid_obstacles.py`

```python
#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('avoider_node')
        
        # Data storage
        self.distances = None
        self.safe_distance = 0.5  # Meters
        
        # Publisher & Subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        self.rate = rospy.Rate(10)

    def lidar_callback(self, msg):
        # Convert tuple to numpy array and clean up 'inf' values
        scan_data = np.array(msg.ranges)
        scan_data[np.isinf(scan_data)] = 3.5
        self.distances = scan_data

    def run(self):
        while not rospy.is_shutdown():
            if self.distances is None:
                continue

            # Define the 'Front' cone (15 degrees left and 15 degrees right)
            # Index 0 is front. Indices 0-15 and 345-359 face forward.
            front_cone = np.concatenate((self.distances[:15], self.distances[-15:]))
            min_dist = np.min(front_cone)

            move_cmd = Twist()

            if min_dist < self.safe_distance:
                rospy.logwarn(f"OBSTACLE DETECTED! Distance: {min_dist:.2f}m")
                # Think: Turn in place
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5 
            else:
                # Think: Move forward
                move_cmd.linear.x = 0.2
                move_cmd.angular.z = 0.0

            # Act:
            self.pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    node = ObstacleAvoider()
    node.run()

```

---

## 4. Part C: Testing in Gazebo

### Step 1: Set up the Course

1. Launch Gazebo with your `lesson1.world` (the one with the walls and poles).
2. Launch the robot bringup (robot_state_publisher).
3. Run your script: `python3 avoid_obstacles.py`.

### Step 2: Debugging with RViz

While the script is running, open **RViz**.

* Observe the **Red Dots** of the Lidar.
* Watch the robot's behavior. Does it stop *before* it hits the wall?
* **Problem Solving:** If the robot hits the wall, is it because the `safe_distance` is too small or because the robot is moving too fast?

---

## 5. Lab Exercises

### Exercise 1: The "Choice" (Left vs. Right)

Currently, the robot only turns **Left** when it sees a wall.
**Task:** Modify the script to look at `self.distances[90]` (Left) and `self.distances[270]` (Right).

* If an obstacle is in front, compare the Left and Right distances.
* Turn toward the side that has **more** free space.

### Exercise 2: Using the `mean` for Smoothness

In some cases, `min()` makes the robot very jittery.
**Task:** Change the logic to use `np.mean()` for the side cones.

* **Question:** Does this make the robot better at navigating through a door? Why might `mean()` be dangerous for very thin poles?

### Exercise 3: The Recovery Behavior

**Challenge:** What if the robot is in a corner and both the front, left, and right are blocked?
**Task:** Add a "Back-up" state. If all three cones are `< 0.3m`, the robot should set `linear.x = -0.1` for 2 seconds.

---

### 🎓 Instructor Notes

* **Asynchronous Data:** Remind students that `lidar_callback` runs in the background. The `run` loop just checks the "latest version" of the data stored in `self.distances`.
* **The "Blind Spot":** Many students forget that Lidar is a 2D slice. If an obstacle is very low (like a laptop on the floor) or very high, the Lidar might not see it, and the robot will crash. This is a great lesson in the limitations of sensors!

---

**This concludes the initial series! Would you like a "Cheat Sheet" of NumPy commands specific to Lidar processing (slicing, filtering, masking) to give to your students?**