import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib import animation
from sim_without_pybullet import simulate_w_scipy

# Define the dimensions of the robot
r = 0.5  # Length of each link

def forward_kinematics(q1, q2):
    x = r * np.cos(q1) + r * np.cos(q1 + q2)
    y = r * np.sin(q1) + r * np.sin(q1 + q2)
    return x, y

# Update function for the animation
def update_plot(i):
    # Compute the forward kinematics for the current joint angles
    x, y = forward_kinematics(q1_sim[i], q2_sim[i])
    # print("(x, y) = " + "(" + str(x) + ", " + str(y) + ")")
    
    # Update the plot objects
    link1.set_data([0, r * np.cos(q1_sim[i])], [0, r * np.sin(q1_sim[i])])
    link2.set_data([r * np.cos(q1_sim[i]), x], [r * np.sin(q1_sim[i]), y])
    joint1.center = (0, 0)
    joint2.center = (r * np.cos(q1_sim[i]), r * np.sin(q1_sim[i]))
    
    # Add the plot objects to the axis
    ax.add_patch(joint1)
    ax.add_patch(joint2)

    return link1, link2, joint1, joint2

# Generate a sequence of joint angles over time
sim_duration = 10
simDT = 1 / 1000
x_init =  np.array( [0.5 * np.pi / 0.6, 0., 0., 0. ] )
# x_init =  np.array( [0., 0., 0., 0. ] )

t, q1, q2 = simulate_w_scipy(x_init, sim_duration, simDT)
q1_sim = [q1[i] + np.pi/2 for i in range(len(q1)) if i % 20 == 0]
q2_sim = [q2[i] for i in range(len(q2)) if i % 20 == 0]
print("q1 dimension: " + str(len(q1_sim)))
print("q2 dimension: " + str(len(q2_sim)))
# print("LQR kicked in: " + str(LQR_kicked_in))

# Create a figure and axis for plotting
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-2*r, 2*r])
ax.set_ylim([-2*r, 2*r])
ax.grid(True)

# Initialize the plot objects for the robot links and joints
link1, = ax.plot([], [], 'b', lw=2)
link2, = ax.plot([], [], 'r', lw=2)
joint1 = Circle((0, 0), 0.1*r, fc='b', ec='b')
joint2 = Circle((0, 0), 0.1*r, fc='r', ec='r')

# Create the animation
print("Simulation starting...")
anim = animation.FuncAnimation(fig, update_plot, frames=len(q1_sim), interval=10)
print("Simulation stopping...")

# Show the animation
plt.show()
