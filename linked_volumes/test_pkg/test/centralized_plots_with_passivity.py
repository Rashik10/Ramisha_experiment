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
k = 20.  #kappa
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
######################################## Optimization Algorithm (Quadprog) (With Passivity) ##################################
##############################################################################################################################

H = np.block([[np.eye(8), np.zeros((8, 4)), np.zeros((8, 2))], [np.zeros((4, 8)), k * np.eye(4), np.zeros((4, 2))], [np.zeros((2, 8)), np.zeros((2, 4)), kh * np.eye(2)]])
f = np.zeros((14,1))

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

while t <= t_end:

  xG = np.zeros((2,1))
  vG = np.zeros((2,1))
  for i in range(4):
    xG = xG + states[0:2,i].reshape(-1,1)
    vG = vG + states[2:4,i].reshape(-1,1)

  xG = (1/4) * xG
  uH = kp * (pG - xG)
  uH_bound = 0.1
  uH[0] = min(uH_bound,max(-uH_bound,uH[0]))
  uH[1] = min(uH_bound,max(-uH_bound,uH[1]))
  vG = (1/4) * vG
  
  
  Jx = np.zeros((4,))
  for i in range(4):
    count1 = 0
    for j in range(4):
      if neighbour[i][j] != 0:
        count1 = count1 + 0.5 * (np.linalg.norm(states[0:2,i] - states[0:2,j]) - neighbour[i][j]) ** 2
    Jx[i] = count1

  Lx = np.zeros((4,))
  for i in range(4):
    Lx[i] = 0.5 * alpha1 * (np.linalg.norm(states[2:4,i])) **2

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
  conv6 = -(uH.T)/4
  eye = (1/4) * np.eye(2)
  A1 = np.block([[dV1[0] @ Bb, zeros, zeros, zeros], [zeros, dV1[1] @ Bb, zeros, zeros], [zeros, zeros, dV1[2] @ Bb, zeros], [zeros, zeros, zeros, dV1[3] @ Bb]])
  A2 = np.block([-1 * np.eye(4)])
  A3 = np.block([[zeros], [zeros], [zeros], [zeros]])
  A4 = np.block([[(states[2:4,0]).T, zeros, zeros, zeros], [zeros, (states[2:4,1]).T, zeros, zeros], [zeros, zeros, (states[2:4,2]).T, zeros], [zeros, zeros, zeros, (states[2:4,3]).T]])
  A5 = np.zeros((4,4))
  A6 = np.block([[conv6], [conv6], [conv6], [conv6]])

  A = np.block([[A1, A2, A3], [A4, A5, A6]])
  b = np.array([-(gama[0] * V1[0]) - (dV1[0] @ Aa @ states[:,0]), -(gama[0] * V1[1]) - (dV1[1] @ Aa @ states[:,1]), -(gama[0] * V1[2]) - (dV1[2] @ Aa @ states[:,2]), -(gama[0] * V1[3]) - (dV1[3] @ Aa @ states[:,3]), 0, 0, 0, 0])
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
  HI = uH + del_uh

  for i in range(4):
    states[:,i] = states[:,i] + (Aa @ states[:,i] + Bb @ u[(i*2):((i*2)+2),0]) * DT

  robot_history = np.dstack((robot_history,states))


  temp_u_i_values = u
  temp_del_uh_i_values = del_uh
  temp_x2_i_values = states[2:4,:].T.flatten()
  temp_x2_i_values = temp_x2_i_values.reshape(-1,1)
  #print(temp_x2_i_values)
  if(t != 0):
    time = np.append(time,t)
    V1_values = np.append(V1_values, V1[0]+V1[1]+V1[2]+V1[3])
    xG_values = np.append(xG_values, xG, axis = 1)
    pG_values = np.append(pG_values, pG, axis = 1)
    velocity_values = np.append(velocity_values, velocity)
    velocity_human_values = np.append(velocity_human_values, velocity_human)
    u_i_values = np.append(u_i_values, temp_u_i_values, axis = 1)
    del_uh_i_values = np.append(del_uh_i_values, temp_del_uh_i_values, axis = 1)
    uH_values = np.append(uH_values, uH, axis = 1)
    HI_values = np.append(HI_values, HI, axis = 1)
    x2_i_values = np.append(x2_i_values, temp_x2_i_values, axis = 1)
  else:
    time = np.array([t])
    V1_values = np.array(V1[0]+V1[1]+V1[2]+V1[3])
    xG_values = xG
    pG_values = pG
    velocity_values = velocity
    velocity_human_values = velocity_human
    u_i_values = temp_u_i_values
    del_uh_i_values = temp_del_uh_i_values
    uH_values = uH
    HI_values = HI
    x2_i_values = temp_x2_i_values
  
  point1.set_data([robot_history[0, 0, -1]], [robot_history[1, 0, -1]])
  point2.set_data([robot_history[0, 1, -1]], [robot_history[1, 1, -1]])
  point3.set_data([robot_history[0, 2, -1]], [robot_history[1, 2, -1]])
  point4.set_data([robot_history[0, 3, -1]], [robot_history[1, 3, -1]])
  point5.set_data([xG_values[0,-1]], [xG_values[1,-1]])
  
  plt.draw()
  plt.pause(0.0001)

  t = t + DT
  print(t)


##############################################################################################################################
###################################################### Plot Formation Graph ##################################################
##############################################################################################################################

#plot the graph for formation
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, V1_values, c="purple", label=f'V1')
    

plt.xlabel('time (s)')
plt.ylabel('V1 (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('xG_and_pG.png', dpi=1200)
#plt.show()

##############################################################################################################################
################################################# Plot xG, pG vs time Graph ##################################################
##############################################################################################################################

#plot the graph for xG, pG vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, xG_values[0], c="red", label=f'xG1')
plt.plot(time, xG_values[1], c="green", label=f'xG2')
plt.plot(time, pG_values[0], c="blue", label=f'pG1')
plt.plot(time, pG_values[1], c="fuchsia", label=f'pG2')
    

plt.xlabel('time (s)')
plt.ylabel('xG and pG (m)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('formation_graph.png', dpi=1200)
#plt.show()

##############################################################################################################################
################################################# Plot Passivity Graph #######################################################
##############################################################################################################################

#plot the graph for passivity
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, velocity_values, c="red", label=f'velocity')
plt.plot(time, velocity_human_values, c="blue", label=f'velocity_human')

plt.ylim(-0.02, 0.02)
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('passivity.png', dpi=1200)
#plt.show()

##############################################################################################################################
################################################# Plot u vs time Graph #######################################################
##############################################################################################################################

#plot the graph of u vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))
for i in range(0,8,2):
    plt.plot(time, u_i_values[i], c=colors[i], label=f'u{int(i/2)+1}1')
    plt.plot(time, u_i_values[i+1], c=colors[i+1], label=f'u{int(i/2)+1}2')

plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right', ncols = 4)
plt.grid(True)

#plt.savefig('u_values.png', dpi=1200)
#plt.show()

##############################################################################################################################
################################################# Plot del_uh vs time Graph ##################################################
##############################################################################################################################

#plot the graph of del_uh vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))

plt.plot(time, del_uh_i_values[0], c=colors[6], label='del_uh1')
plt.plot(time, del_uh_i_values[1], c=colors[7], label='del_uh2')

plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right', ncols = 1)
plt.grid(True)

#plt.savefig('del_uh_values.png', dpi=1200)
#plt.show()

##############################################################################################################################
########################################## Plot uH, uH+del_uH vs time Graph ##################################################
##############################################################################################################################

#plot the graph for uH, uH+del_uH vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, uH_values[0], c="red", label=f'uH1')
plt.plot(time, uH_values[1], c="blue", label=f'uH2')
plt.plot(time, HI_values[0], c="green", label=f'uH1+del_uH1')
plt.plot(time, HI_values[1], c="brown", label=f'uH2+del_uH2')

plt.xlabel('time (s)')
plt.ylabel('uH and uH+del_uH (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('uH and uH+del_uH.png', dpi=1200)
#plt.show()

##############################################################################################################################
################################################# Plot x2 vs time Graph ######################################################
##############################################################################################################################

#plot the graph of x2 vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))

for i in range(0,8,2):
  plt.plot(time, x2_i_values[i], c=colors[i], label=f'x{int(i/2)+1}21')
  plt.plot(time, x2_i_values[i+1], c=colors[i+1], label=f'x{int(i/2)+1}22')

plt.xlabel('time (s)')
plt.ylabel('x2 (speed)(m/s)')
plt.legend(loc = 'upper right', ncols = 4)
plt.grid(True)

#plt.savefig('x2_values.png', dpi=1200)
plt.show()

##############################################################################################################################
################################################################ FOR ANIMATION ###############################################
##############################################################################################################################