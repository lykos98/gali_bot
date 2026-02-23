To get **SLAM + Auto-exploration** running correctly, you need to launch the components in a specific sequence so the coordinate frames (`map` → `odom` → `base_link`) and the Action Server namespaces align properly.

Based on your current setup where `move_base` is running as `move_base_node`, here is the complete command list:

---

### 1. Terminal 1: Simulation

Ensure Gazebo is running and the robot is spawned.

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

```

### 2. Terminal 2: SLAM (The Map Builder)

This node creates the `map` frame and provides the data `explore_lite` needs to find frontiers.

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_gmapping.launch

```

### 3. Terminal 3: Navigation (The Legs)

We will launch this with a name remap to ensure it uses the standard `/move_base` namespace that exploration nodes expect.

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch move_base_node:=move_base

```

> **Note:** Watch this terminal. If you see "Robot is off the costmap," the robot is at a coordinate (like your -2.0) that the current map doesn't cover yet.

### 4. Terminal 4: Explore Lite (The Brain)

Launch the exploration node. If you renamed the node in the previous step to `move_base`, use this standard command:

```bash
roslaunch explore_lite explore.launch

```

*If you did **not** rename the navigation node and it is still running as `move_base_node`, use this remapped version instead:*

```bash
roslaunch explore_lite explore.launch move_base:=move_base_node explore/costmap:=/move_base_node/global_costmap/costmap

```

### 5. Terminal 5: The "Reset" (Only if it doesn't move)

If everything is running but the robot is stuck, force the costmaps to update their origin to the robot's current position:

```bash
rosservice call /move_base/clear_costmaps "{}"

```