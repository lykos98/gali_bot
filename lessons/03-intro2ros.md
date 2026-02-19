Here is the complete, formatted Markdown for your first lesson. You can copy and paste this directly into a `.md` file for your students.

---

# Lesson 1: Foundations of ROS & Gazebo

## 1. Objectives

* Understand the **Master-Node** architecture.
* Launch a physical robot simulation in **Gazebo**.
* Master the Command Line Interface (CLI) to inspect and move a robot.

---

## 2. Part A: The "Brain" of ROS (Master and Nodes)

ROS (Robot Operating System) is a distributed system. Nodes are small programs that talk to each other. For them to communicate, a "phonebook" called the **Master** must be running.

### Step 1: Start the Master

Open your terminal and type:

```bash
roscore

```

> **Keep this terminal open!** If you close it, the "brain" dies, and nodes can no longer find each other.

### Step 2: The "Bringup"

A **Bringup** is a set of scripts that initializes the robot's hardware or its digital twin. Since we are using a TurtleBot3, we must define the model first.

Open a **new terminal** and run:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_remote.launch

```

---

## 3. Part B: The "World" (Gazebo Simulator)

Gazebo is a 3D physics engine. It simulates gravity, friction, and sensors like LiDAR.

### Step 1: Launch the Environment

Open a **new terminal** and run:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

```

### Step 2: Familiarizing with Gazebo

* **Left Click:** Select objects.
* **Mouse Scroll:** Zoom.
* **Middle Click (Wheel):** Rotate camera.
* **Play/Pause Button:** Found at the bottom. If the simulation is paused, the robot will **not** move.

This is a great addition for the lesson. Students often find Gazebo's world-building tools intuitive but can get frustrated when objects don't behave as expected (e.g., walls falling over or disappearing).

Here is the updated section specifically focused on **Environment Customization** in Gazebo. You can append this to your Lesson 1 notes.

---

## 3. Part B (Extended): The "World" & Environment Building

Gazebo is not just for viewing; it is a **CAD-lite** environment where you can build obstacle courses for your robot.

### 1. Placing 3D Shapes (Basic Obstacles)

At the top of the Gazebo window, you will see a toolbar with geometric icons:

* **Cube, Sphere, Cylinder:** Click one, then click anywhere on the ground plane to "spawn" it.
* **Selection Tool (ESC):** Use this to select an existing object.
* **Translate Tool (T):** Drag the **Red (X)**, **Green (Y)**, or **Blue (Z)** arrows to move the object.
* **Rotate Tool (R):** Use the rings to tilt or spin the object.
* **Scale Tool (S):** Resize the object (useful for making long thin blocks to act as walls).

### 2. Building Walls (The "Building Editor")

For professional-looking walls and rooms, do not use stretched cubes. Use the **Building Editor**:

1. Go to **Edit -> Building Editor** (or press `Ctrl+B`).
2. **2D View:** On the top-left panel, select **Wall**. Click and drag on the 2D grid to "draw" your floor plan.
3. **Features:** You can add **Windows** and **Doors** by dragging them onto your drawn walls.
4. **Textures:** Select the **Brick** or **Wood** icons to change the wall appearance.
5. **Importing to Sim:** When finished, go to **File -> Exit Building Editor**. Gazebo will ask if you want to save; click **Yes**, and the walls will appear in your 3D world.

### 3. The "Insert" Tab (Ready-Made Models)

On the left-side panel, click the **Insert** tab.

* This contains a library of pre-built models (Houses, Traffic Lights, Trash bins, etc.).
* **Note:** The first time you click a model, Gazebo might "freeze" for a moment while it downloads the files from the online Fuel server.

---

## 4. Part B.2: Physics & The "Scene" Tree

Understanding the **Left Panel (World Tab)** is crucial for debugging:

* **The Model List:** Every object you place appears here. If you "lose" your robot in the world, double-click its name in this list to zoom the camera directly to it.
* **Static vs. Dynamic:** * If you place a wall and the robot knocks it over, the wall is **Dynamic**.
* To make a wall "immovable," right-click the object -> **Open Link Inspector** -> Check the **Static** box.


* **Gravity:** You can turn gravity off in the "Physics" properties, but be careful—your robot might float away!

---

## 5. Lab Exercise: The "Obstacle Course" Challenge

**Goal:** Build a custom environment and test the robot's sensors.

1. **The Maze:** Use the **Building Editor** to create a small "L-shaped" corridor.
2. **The "Nope" Wall:** Place a single **Cylinder** directly in front of the robot.
3. **Observation:** Open a terminal and run `rostopic echo /scan`.
* Look at the data values while you move the Cylinder closer to and further from the robot in the Gazebo UI.
* Notice how the numbers in the `ranges` array change in real-time.


4. **Challenge:** Try to stack two cubes. Why is it difficult? (Hint: Check the **Grid Snapping** settings at the top).

---

### 🎓 Instructor Note on Saving

Remind students that Gazebo **does not autosave**.

* If they build a complex world, they must go to **File -> Save World As** and save it as a `.world` file.
* To launch that world next time: `gazebo my_world.world`.

**Would you like me to provide a "Starter World" file code that you can give to students, containing a pre-made maze and some obstacles?**

---

## 4. Part C: Movement and Inspection

Now that the robot is "awake" in the world, we need to talk to it via **Topics**.

### Step 1: Inspecting Topics

Topics are channels where data flows. List all active channels:

```bash
rostopic list

```

* `/cmd_vel`: The channel for Velocity Commands (Input).
* `/scan`: The channel for LiDAR Laser Data (Output).
* `/odom`: The channel for Position/Odometry (Output).

### Step 2: Manual Movement (Teleop)

Open a **new terminal** and run the keyboard controller:

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

```

* Use **W, A, S, D, X** to move.
* Keep this terminal active/focused to control the robot.

---

## 5. Lab Exercises

### Exercise 1: The "Manual Square"

1. Start the `teleop` node.
2. Attempt to drive the robot in a **perfect  square** using only your eyes.
3. **Question:** Why is it difficult to make a perfect  turn? How did the robot’s "momentum" affect your accuracy?

### Exercise 2: Topic Inspection & Message Anatomy

1. Check the message type of the velocity topic: `rostopic type /cmd_vel`.
2. See the internal fields of that message: `rosmsg show geometry_msgs/Twist`.
3. In a separate terminal, run `rostopic echo /cmd_vel`.
4. **Task:** Record the `linear.x` value at max speed. Record the `angular.z` value during a sharp turn.

### Exercise 3: Command Line Injection (The "Ghost" Command)

1. Close the teleop node.
2. We can move the robot without a script! Use the `rostopic pub` command to make the robot spin in place at  (10 times per second):
```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

```


3. **Challenge:** Modify the command above to make the robot drive in a continuous circle with a radius of . (*Hint: *).

### Exercise 4: Visualizing the Network

1. Run `rosrun rqt_graph rqt_graph`.
2. Identify which node is "talking" to `/cmd_vel` and which node is "listening" to it.

---