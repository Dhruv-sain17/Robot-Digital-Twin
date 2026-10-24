
import heapq
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

#  Define the environment
WORLD_WIDTH = 10
WORLD_HEIGHT = 10

#  Define the obstacle: (x, y, radius)
obstacles = [
    (4.5, 1.0, 0.6),
    (6.5, 3.0, 0.6),
    (7.5, 6.0, 0.8)
]
# Velocity of the moving obstacle (units per step)
moving_obstacle_index = 1
obstacle_velocity = (0.0, 0.1)
obstacle_history = []

#  Set the robot's starting position
robot_x = 1.0
robot_y = 1.0

SENSOR_RANGE = 2.0
STEP_SIZE = 0.5
AVOIDANCE_SHIFT = 2.0
# Store the robot's path and sensor detections
path_x = [robot_x]
path_y = [robot_y]
sensor_detections = []
obstacle_detections = 0

def front_sensor(x, y):
    """Return the distance to the nearest obstacle directly ahead."""
    nearest_distance = SENSOR_RANGE

    for cx, cy, radius in obstacles:
        # This simple sensor points directly to the right.
        vertical_offset = abs(y - cy)

        if vertical_offset >= radius or cx <= x:
            continue

        # Calculate where the sensor ray meets the circle.
        horizontal_offset = (
            radius**2 - vertical_offset**2
        ) ** 0.5

        distance = cx - horizontal_offset - x

        if 0 <= distance < nearest_distance:
            nearest_distance = distance

    return nearest_distance

def is_collision(x, y):
    """Check whether a position overlaps an obstacle."""
    for cx, cy, radius in obstacles:
        dx = x - cx
        dy = y - cy
        distance_to_center = (dx**2 + dy**2) ** 0.5

        if distance_to_center <= radius:
            return True

    return False

def is_collision(x, y):
    """Return True if the robot's position overlaps an obstacle."""
    for cx, cy, radius in obstacles:
        dx = x - cx
        dy = y - cy
        distance_to_center = (dx**2 + dy**2) ** 0.5

        if distance_to_center <= radius:
            return True

    return False

def path_has_collision(x1, y1, x2, y2):
    """Check whether a movement segment intersects an obstacle."""
    for cx, cy, radius in obstacles:
        dx = x2 - x1
        dy = y2 - y1

        segment_length_squared = dx**2 + dy**2

        if segment_length_squared == 0:
            closest_x = x1
            closest_y = y1
        else:
            t = (
                (cx - x1) * dx + (cy - y1) * dy
            ) / segment_length_squared

            t = max(0, min(1, t))

            closest_x = x1 + t * dx
            closest_y = y1 + t * dy

        distance = (
            (cx - closest_x)**2 + (cy - closest_y)**2
        ) ** 0.5

        if distance <= radius:
            return True

    return False


#  Plan a collision-aware route using A*

GRID_SIZE = 0.25
SAFETY_MARGIN = 0.25
MAX_GRID_X = int(WORLD_WIDTH / GRID_SIZE)
MAX_GRID_Y = int(WORLD_HEIGHT / GRID_SIZE)

START = (round(robot_x / GRID_SIZE), round(robot_y / GRID_SIZE))
GOAL_X = 9.0
GOAL_Y = 5.0
GOAL = (round(GOAL_X / GRID_SIZE), round(GOAL_Y / GRID_SIZE))

def grid_to_world(node):
    return node[0] * GRID_SIZE, node[1] * GRID_SIZE

def is_blocked(node):
    x, y = grid_to_world(node)

    if not (0 <= node[0] <= MAX_GRID_X
            and 0 <= node[1] <= MAX_GRID_Y):
        return True

    for cx, cy, radius in obstacles:
        distance = ((x - cx)**2 + (y - cy)**2) ** 0.5

        if distance <= radius + SAFETY_MARGIN:
            return True

    return False


def move_obstacle():
    global obstacle_velocity

    cx, cy, radius = obstacles[moving_obstacle_index]
    vx, vy = obstacle_velocity

    next_y = cy + vy
    obstacles[moving_obstacle_index] = (cx + vx, next_y, radius)
    obstacle_history.append(obstacles[moving_obstacle_index])

    # Reverse direction at the safe vertical boundaries.
    if (next_y >= WORLD_HEIGHT - radius - SAFETY_MARGIN
            or next_y <= radius + SAFETY_MARGIN):
        vy = -vy
        next_y = cy + vy
        obstacle_velocity = (vx, vy)

    


def heuristic(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx**2 + dy**2) ** 0.5

def find_path(start, goal):
    open_set = [(heuristic(start, goal), 0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    visited = set()

    directions = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path, len(visited)

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if is_blocked(neighbor) or neighbor in visited:
                continue

            # Prevent diagonal movement through obstacle corners.
            if dx != 0 and dy != 0:
                if (is_blocked((current[0] + dx, current[1]))
                        or is_blocked((current[0], current[1] + dy))):
                    continue

            move_cost = 1.41421356 if dx != 0 and dy != 0 else 1.0
            new_cost = current_cost + move_cost

            if new_cost < cost_so_far.get(neighbor, float("inf")):
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(
                    open_set, (priority, new_cost, neighbor)
                )

    return None, len(visited)

if is_blocked(START):
    raise ValueError("The robot's starting position is blocked.")

if is_blocked(GOAL):
    raise ValueError("The goal position is blocked.")



planned_path, nodes_explored = find_path(START, GOAL)

if planned_path is None:
    raise RuntimeError("No safe route to the goal was found.")


# Convert the planned grid route into world coordinates.
path_points = [grid_to_world(node) for node in planned_path]

# The robot starts at the beginning of the route.
robot_x, robot_y = path_points[0]

# Record the positions the robot actually visits.
path_x = [robot_x]
path_y = [robot_y]

# Count replanning events.
replanning_count = 0




# Dynamic obstacle avoidance with automatic replanning
current_node = START
max_steps = 500

for step in range(max_steps):
    if current_node == GOAL:
        break

    # Move the obstacle.
    move_obstacle()

    # Check whether the remaining route is blocked.
    remaining_path = planned_path[
        planned_path.index(current_node) + 1:
    ]

    route_blocked = any(is_blocked(node) for node in remaining_path)

    if route_blocked:
        new_path, explored = find_path(current_node, GOAL)
        nodes_explored += explored

        if new_path is None:
            print("No safe route available. Robot stopped.")
            break

        planned_path = new_path
        replanning_count += 1
        print(f"Replanned route! Event #{replanning_count}")

    # Check whether the robot's current position is blocked.
    if is_blocked(current_node):
        print("WARNING: Robot position is blocked!")
        break

    # Move one step along the current route.
    if current_node == GOAL:
        break

    current_index = planned_path.index(current_node)

    if current_index + 1 >= len(planned_path):
        break

    next_node = planned_path[current_index + 1]

    # Never enter a grid cell currently blocked by an obstacle.
    if is_blocked(next_node):
        continue

    current_node = next_node
    robot_x, robot_y = grid_to_world(current_node)
    path_x.append(robot_x)
    path_y.append(robot_y)
else:
    print("Maximum movement steps reached.")




# A* plans around obstacles rather than relying on reactive sensor turns.
obstacle_detections = 0
sensor_detections = []

print(f"A* route planned successfully: {len(path_x)} positions")
print(f"Grid nodes explored: {nodes_explored}")




#  Visualize the simulation
fig, ax = plt.subplots(figsize=(8, 8))


# Draw the goal area
GOAL_X = 9.0
GOAL_Y = 5.0
GOAL_TOLERANCE = 0.5

goal_area = Circle(
    (GOAL_X, GOAL_Y),
    GOAL_TOLERANCE,
    color="green",
    alpha=0.25,
    label="Goal area"
)
ax.add_patch(goal_area)

# Draw obstacles
for cx, cy, radius in obstacles:
    ax.add_patch(
        Circle(
            (cx, cy), radius,
            color="red", alpha=0.6,
            label="Obstacle"
        )
    )

# Draw the robot's path
ax.plot(path_x, path_y, "b-o", markersize=3, label="Robot path")

# Mark start and finish
ax.scatter(path_x[0], path_y[0], color="green", s=100, label="Start")
ax.scatter(path_x[-1], path_y[-1], color="black", s=100, label="Finish")

# Show sensor readings where obstacles were detected
for x, y, distance in sensor_detections:
    ax.plot(
        [x, x + distance], [y, y],
        color="orange", linestyle="--",
        linewidth=2, label="Sensor detection"
    )

ax.set_xlim(0, WORLD_WIDTH)
ax.set_ylim(0, WORLD_HEIGHT)
ax.set_aspect("equal")
ax.set_xlabel("X position")
ax.set_ylabel("Y position")
ax.set_title("Robot Simulation: Obstacle Avoidance")
ax.grid(True)
ax.legend()
plt.show()
# Calculate total distance travelled
total_distance = 0.0

for i in range(1, len(path_x)):
    dx = path_x[i] - path_x[i - 1]
    dy = path_y[i] - path_y[i - 1]

    total_distance += (dx**2 + dy**2) ** 0.5



# Define the goal area
GOAL_X = 9.0
GOAL_Y = 5.0
GOAL_TOLERANCE = 0.5

# Check whether the robot reached the goal area
goal_reached = (
    abs(robot_x - GOAL_X) <= GOAL_TOLERANCE
    and abs(robot_y - GOAL_Y) <= GOAL_TOLERANCE
)



# Check recorded positions and movement segments
collision_detected = any(
    is_collision(x, y)
    for x, y in zip(path_x, path_y)
)

for i in range(1, len(path_x)):
    if path_has_collision(
        path_x[i - 1], path_y[i - 1],
        path_x[i], path_y[i]
    ):
        collision_detected = True
        break



print("\n--- Robot Performance Report ---")
print(f"Total movement steps: {len(path_x) - 1}")
print(f"Distance travelled: {total_distance:.2f} units")

# Calculate straight-line distance from start to finish
start_x, start_y = path_x[0], path_y[0]
straight_line_distance = (
    (robot_x - start_x)**2 + (robot_y - start_y)**2
) ** 0.5

# Calculate path efficiency
if total_distance > 0:
    path_efficiency = (
        straight_line_distance / total_distance
    ) * 100
else:
    path_efficiency = 0.0

print(f"Path efficiency: {path_efficiency:.2f}%")

print(f"Replanning events: {replanning_count}")
print(f"Obstacle positions recorded: {len(obstacle_history)}")

# Check the robot against the moving obstacle at each recorded step.
dynamic_collision = False

for i, (rx, ry) in enumerate(zip(path_x[1:], path_y[1:])):
    if i >= len(obstacle_history):
        break

    ox, oy, radius = obstacle_history[i]
    distance = ((rx - ox)**2 + (ry - oy)**2) ** 0.5

    if distance <= radius:
        dynamic_collision = True
        print(f"Dynamic collision at movement step {i + 1}")
        break

print(f"Dynamic collision detected: {dynamic_collision}")

print("\n--- Dynamic Obstacle Summary ---")
print(f"Obstacle movement steps: {len(obstacle_history)}")
print(f"Automatic replans: {replanning_count}")
print(f"Robot reached goal: {goal_reached}")
print(f"Dynamic collision in sampled positions: {dynamic_collision}")

print(f"Final position: ({robot_x:.2f}, {robot_y:.2f})")
print(f"Goal reached: {goal_reached}")
print(f"Collision detected: {collision_detected}")
