# ROS Projects - Turtle Pong Simulator (ROS 2)

This is my **first project using ROS 2**, where I attempted to build a simple **ping pong simulator** using the `turtlesim` package. The game is controlled using **two players on the keyboard**, each controlling a paddle to hit the ball.

---

## Features

- Two-player ping pong game using keyboard controls.
- Ball moves automatically and bounces off paddles and walls.
- Built entirely using ROS 2 nodes in Python (`rclpy`) and `turtlesim`.

---

## Current Limitations

- **No scoreboard yet**: Currently, you cannot see the score. A dashboard for tracking points will be added in future updates.
- **No launch file**: Nodes need to be run manually. A launch file will allow starting all nodes with a single command.
- **User interface**: Ideally, the game will have buttons or interactive controls, but this is not implemented yet.

---

## Package Structure

- `my_package/` – Contains all Python nodes for the game.
  - `spawn_and_start.py` – Spawns turtles and starts the game.
  - `paddle_teleop.py` – Handles paddle control for players.
  - `ball_node.py` – Handles ball movement and collision logic.
- `launch/` – Placeholder for future launch files.
- `setup.py` – Package installation configuration.
- `package.xml` – ROS 2 package manifest.

---

## How to Run

### 1. Source your workspace

```bash
source install/setup.bash
```


### 2. Start the turtlesim node
### 3. Run the game nodes

```bash
ros2 run my_package spawn_and_start
ros2 run my_package paddle_teleop
ros2 run my_package ball_node
```

### Future Plans
* Create a scoreboard/dashboard to track player points.

* Build a launch file to start all nodes with one command.

* Explore adding UI buttons or controls for easier gameplay.

* Improve ball physics and paddle movement.


### Notes
* This project is primarily a learning exercise in ROS 2 and turtlesim, aiming to understand:

* ROS 2 nodes, topics, and multi-node communication.

* Keyboard teleoperation for multiple turtles.

* Structuring Python packages for ROS 2.