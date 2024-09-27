##############################################################################################################################
###################################################### Libraries #############################################################
##############################################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from cvxopt import solvers, matrix
import matplotlib.animation as animation

##############################################################################################################################
###################################################### Variable Initialization ###############################################
##############################################################################################################################

# Simulation parameters
DT = 0.02
t_end = 100

# Constant parameters
k = 10.  #kappa
kh = 1.
kv = 0.1
kp = 0.1
#K = 100

# Function to generate random positions within specified bounds
#def random_positions(n, x_min, x_max, y_min, y_max):
#    x = np.random.uniform(x_min, x_max, n)
#    y = np.random.uniform(y_min, y_max, n)
#    return np.vstack((x, y))

# Number of robots
#num_robots = 4
#start_position = random_positions(num_robots, 0.5, 1.5, -1.5, -0.5)

start_position = np.array([[1.5, 0.5, 1.5, 0.5],
                           [-1.5, -1.5, -0.5, -0.5]])
vel = np.array([[0., 0., 0., 0.],
                [0., 0., 0., 0.]])

# Initialize robot position and velocity
states = np.append(start_position, vel, axis=0)

t = 0
gama = np.array([0.3,0.1])
alpha1 = 2.

Aa = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
Bb = np.block([[np.zeros((2, 2))], [np.eye(2)]])

pG= np.array([[-0.75], [0.75]])
#vec4 = np.array([[0], [0]])

neighbour = np.array([[0, 0.8, 0.8, 1.13],
                      [0.8, 0, 1.13, 0.8],
                      [0.8, 1.13, 0, 0.8],
                      [1.13, 0.8, 0.8, 0]])

# Create lists to store robot states for plotting
robot_history = states

##############################################################################################################################
################################### Optimization Algorithm (Quadprog) (Without Passivity) ####################################
##############################################################################################################################

H = np.block([[np.eye(8), np.zeros((8, 4)), np.zeros((8, 2))], [np.zeros((4, 8)), k * np.eye(4), np.zeros((4, 2))], [np.zeros((2, 8)), np.zeros((2, 4)), kh * np.eye(2)]])
f = np.zeros((14,1))

while t <= t_end:

  xG = np.zeros((2,1))
  vG = np.zeros((2,1))
  for i in range(4):
    xG = xG + states[0:2,i].reshape(-1,1)
    vG = vG + states[2:4,i].reshape(-1,1)

  xG = (1/4) * xG
  uH = kp * (pG - xG)
  uH[0] = min(0.1,max(-0.1,uH[0]))
  uH[1] = min(0.1,max(-0.1,uH[1]))
  vG = (1/4) * vG
  
  
  Jx = 0
  for i in range(4):
    count1 = 0
    for j in range(4):
      if neighbour[i][j] != 0:
        count1 = count1 + 0.5 * (np.linalg.norm(states[0:2,i] - states[0:2,j]) - neighbour[i][j]) ** 2
    Jx = Jx + count1

  Lx = 0
  for i in range(4):
    Lx = Lx + 0.5 * alpha1 * (np.linalg.norm(states[2:4,i])) **2

  V1 = Jx + Lx

  dJxi = np.zeros((4,2))
  for i in range(4):
    temp2 = np.zeros((1,2))
    for j in range(4):
      if neighbour[i][j] != 0:
        temp1 = np.linalg.norm(states[0:2,i] - states[0:2,j])
        temp2 = temp2 + ((temp1 - neighbour[i][j]) / temp1) * (states[0:2,i] - states[0:2,j]).T
    dJxi[i] = 2 * temp2

  dV1 = np.block([dJxi, alpha1 * ((states[2:4]).T)])

  zeros = np.zeros((1,2))
  eye = (1/4) * np.eye(2)
  A1 = np.block([[dV1[0] @ Bb, zeros, zeros, zeros], [zeros, dV1[1] @ Bb, zeros, zeros], [zeros, zeros, dV1[2] @ Bb, zeros], [zeros, zeros, zeros, dV1[3] @ Bb]])
  A2 = np.block([-1 * np.eye(4)])
  A3 = np.block([[zeros], [zeros], [zeros], [zeros]])

  A = np.block([A1, A2, A3])
  b = np.array([-(gama[0] * V1) - (dV1[0] @ Aa @ states[:,0]), -(gama[0] * V1) - (dV1[1] @ Aa @ states[:,1]), -(gama[0] * V1) - (dV1[2] @ Aa @ states[:,2]), -(gama[0] * V1) - (dV1[3] @ Aa @ states[:,3])])
  b = b.reshape(-1,1)
  C = np.block([eye, eye, eye, eye, np.zeros((2, 4)), -kv * np.eye(2)])
  d = kv * uH - kv * vG
  d = d.reshape(-1,1)

  # Solve quadratic programming problems using cvxopt.solve_qp for u
  solvers.options['show_progress'] = False  # Hide progress output
  v = solvers.qp(matrix(H), matrix(f), matrix(A), matrix(b), matrix(C), matrix(d))

  u = np.array(v['x'])[0:8]

  delt = np.array(v['x'])[8:12]

  del_uh = np.array(v['x'])[12:14]

  #temp_vec4 = np.array(u4['x'])[4:6]
  #vec4 = np.append(vec4, temp_vec4, axis=1)

  velocity = states[2:4,:].T.flatten() @ u
  velocity_human = (uH.T) @ del_uh

  for i in range(4):
    states[:,i] = states[:,i] + (Aa @ states[:,i] + Bb @ u[(i*2):((i*2)+2),0]) * DT

  robot_history = np.dstack((robot_history,states))


  if(t != 0):
    time = np.append(time,t)
    V1_values = np.append(V1_values, V1)
    xG_values = np.append(xG_values, xG, axis = 1)
    velocity_values = np.append(velocity_values, velocity)
    velocity_human_values = np.append(velocity_human_values, velocity_human)
  else:
    time = np.array([t])
    V1_values = np.array([V1])
    xG_values = xG
    velocity_values = velocity
    velocity_human_values = velocity_human

  t = t + DT

##############################################################################################################################
################################################################ FOR ANIMATION ###############################################
##############################################################################################################################

# Animation Code 
# Define the points of the fixed quadrilateral
quad_points = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

colors = np.array(["red","blue","green","brown"])

# Create the figure and axis
fig, ax = plt.subplots(figsize=(8, 8))
point1, = ax.plot([], [], 'o', color=colors[0], label='Robot 1')
point2, = ax.plot([], [], 'o', color=colors[1], label='Robot 2')
point3, = ax.plot([], [], 'o', color=colors[2], label='Robot 3')
point4, = ax.plot([], [], 'o', color=colors[3], label='Robot 4')
point5, = ax.plot([], [], 'x', color="purple", label='xG')
point6, = ax.plot([pG[0,0]], [pG[1,0]], 'x', color=colors[3], label='pG')

# Plot the fixed quadrilateral
quad = plt.Polygon(quad_points, closed=True, edgecolor='C9', facecolor='none', label='Fixed Quadrilateral')

# Set the axis limits
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.grid(True)  # Add grid lines

# Add legend
ax.legend()

# Add the quadrilateral to the plot
ax.add_patch(quad)

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Robot Motion')

# Set the aspect ratio to be equal
ax.set_aspect('equal')

# Animation function: this is called sequentially
def animate(i):
    point1.set_data([robot_history[0, 0, i]], [robot_history[1, 0, i]])
    point2.set_data([robot_history[0, 1, i]], [robot_history[1, 1, i]])
    point3.set_data([robot_history[0, 2, i]], [robot_history[1, 2, i]])
    point4.set_data([robot_history[0, 3, i]], [robot_history[1, 3, i]])
    point5.set_data([xG_values[0,i]], [xG_values[1,i]])
    return point1, point2, point3,point4, point5, point6, quad

# Create the animation
ani = animation.FuncAnimation(fig, animate, frames = robot_history.shape[2]-1, interval=5, blit=True)

# Save the animation
#ani.save('animation.gif', writer='pillow')
#ani.save('animation.mp4', writer='ffmpeg')

# Display the animation
plt.show()

##############################################################################################################################
################################################################ Formation Graph #############################################
##############################################################################################################################

#plot the graph for formation
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, V1_values, c="purple", label=f'V1')
    

plt.xlabel('time (s)')
plt.ylabel('V1 (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('u_values.png', dpi=1200)
plt.show()

##############################################################################################################################
################################################################ Passivity Graph #############################################
##############################################################################################################################

#plot the graph for passivity
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, velocity_values, c="red", label=f'velocity')
plt.plot(time, velocity_human_values, c="blue", label=f'velocity_human')

plt.ylim(-0.1, 0.1)
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('u_values.png', dpi=1200)
plt.show()


##############################################################################################################################
################################################# Plot x2 vs time Graph ######################################################
##############################################################################################################################

#plot the graph of x2 vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))

for i in range(4):
  plt.plot(time, robot_history[2, i, :-1], c=colors[i], label=f'x{i+1}21')
  plt.plot(time, robot_history[3, i, :-1], c=colors[i+1], label=f'x{i+1}22')

plt.xlabel('time (s)')
plt.ylabel('x2 (speed)(m/s)')
plt.legend(loc = 'upper right', ncol = 4)
plt.grid(True)

#plt.savefig('x2_values.png', dpi=1200)
plt.show()




