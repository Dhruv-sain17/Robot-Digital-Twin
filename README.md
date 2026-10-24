Robot Digital Twin: Path Planning and Dynamic Obstacle Avoidance

A Python simulation of a robot finding its way through a 2D environment full of obstacles. It uses the A* algorithm to plan a route, and it can handle a moving obstacle by replanning on the fly whenever the path gets blocked.

What It Does
Sets up a 2D environment with a start point and a goal
Plans routes on a grid using A*
Keeps the robot clear of obstacles with an adjustable safety margin
Moves an obstacle back and forth, reversing direction when it hits a boundary
Replans automatically from the robot’s current position if the route ahead gets blocked
Reports how the run went: distance travelled, path efficiency, and number of replans
Checks for collisions at sampled points along the way
Plots the environment and the robot’s path with Matplotlib
Built With
Python
A* search
Matplotlib
NumPy (where the current implementation uses it)
How It Works

The environment is a grid. A* finds a route from the start to the goal, treating obstacles and their safety margins as off-limits.

While the robot moves, one obstacle keeps moving too. If it ends up on the robot’s remaining route, the planner finds a fresh path from wherever the robot currently is to the goal. When the run ends, the simulator reports how well the robot did and whether it reached the goal.

Sample Results

Here’s what the current configuration produces:

Distance travelled: 9.80 units
Path efficiency: 91.24%
Replanning events: 5
Obstacle positions recorded: 33
Goal reached: Yes
Collision in sampled positions: None detected

These numbers will shift if you change the environment, grid resolution, obstacle movement, or planning parameters.

Getting Started

Install the dependencies:

pip install -r requirements.txt

Then run the simulator:

python main.py
Limitations

This is a software simulation, not a physical robot or a full real-time digital twin. The robot works from the simulated environment instead of live sensor data. Collision checking only looks at sampled positions, so it can’t guarantee the robot never hits something between those samples.

What’s Next
Animate the robot and moving obstacles in real time
Check for collisions between consecutive robot positions, not just at sampled points
Add simulated sensors so the robot detects nearby obstacles locally
Try different obstacle layouts and starting positions
Compare path planning performance across multiple scenarios