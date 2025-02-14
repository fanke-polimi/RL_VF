import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Parameters
link_length = 1  # Length of each link
num_links = 3  # Number of links
angles = [np.pi / 6, np.pi / 4, np.pi / 3]  # Initial joint angles

target = np.array([2, 1])  # Target position
learning_rate = 0.08  # Reduced learning rate for stability

def forward_kinematics(angles):
    """Compute the x, y positions of the robot arm."""
    x, y = [0], [0]  # Start at the origin
    for angle in angles:
        x.append(x[-1] + link_length * np.cos(angle))
        y.append(y[-1] + link_length * np.sin(angle))
    return np.array([x, y])

def compute_jacobian(angles):
    """Compute the Jacobian matrix."""
    J = np.zeros((2, len(angles)))
    x, y = 0, 0
    total_angle = 0
    for i in range(len(angles)):
        total_angle += angles[i]
        x += link_length * np.cos(total_angle)
        y += link_length * np.sin(total_angle)
        for j in range(i + 1):
            J[0, j] -= link_length * np.sin(total_angle)
            J[1, j] += link_length * np.cos(total_angle)
    return J

def inverse_kinematics(target, angles, iterations=20):
    """Iteratively adjust angles to reach the target using the Jacobian Transpose Method."""
    for _ in range(iterations):
        positions = forward_kinematics(angles)
        end_effector = positions[:, -1]
        error = target - end_effector

        # Stop if the error is small
        if np.linalg.norm(error) < 1e-2:
            break

        # Compute the Jacobian and update angles using Jacobian Transpose
        J = compute_jacobian(angles)
        d_theta = 0.01*J.T @ error

        # Scale the update to avoid large changes
        max_angle_step = 0.1
        d_theta = np.clip(d_theta, -max_angle_step, max_angle_step)

        angles += learning_rate * d_theta
    return angles

def is_target_reachable(target):
    """Check if the target is within the arm's reachable space."""
    max_reach = num_links * link_length
    return np.linalg.norm(target) <= max_reach

def reset_target_and_arm():
    """Reset the target and arm to random positions."""
    global target, angles
    target = np.random.uniform(-num_links * link_length, num_links * link_length, size=2)
    angles = np.random.uniform(-np.pi, np.pi, size=num_links)

# Initialize plot
fig, ax = plt.subplots()
ax.set_xlim(-num_links * link_length, num_links * link_length)
ax.set_ylim(-num_links * link_length, num_links * link_length)
ax.set_aspect('equal')
ax.grid(True)

# Line for the robot arm
line, = ax.plot([], [], 'o-', lw=2, color='blue')

# Target marker
target_marker, = ax.plot([target[0]], [target[1]], 'g*', markersize=15)

# Animation update function
def update(frame):
    global angles, target
    if not is_target_reachable(target):
        reset_target_and_arm()

    angles = inverse_kinematics(target, angles, iterations=20)
    positions = forward_kinematics(angles)
    line.set_data(positions[0], positions[1])

    # Check if the arm reached the target
    end_effector = positions[:, -1]
    if np.linalg.norm(target - end_effector) < 1e-2:
        reset_target_and_arm()
        target_marker.set_data([target[0]], [target[1]])

    return line, target_marker

# Animation
ani = FuncAnimation(fig, update, frames=range(200), interval=50, blit=True)

plt.show()
