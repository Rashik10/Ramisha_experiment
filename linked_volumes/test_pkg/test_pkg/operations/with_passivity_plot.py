import yaml
import numpy as np
import matplotlib.pyplot as plt

# Load the YAML file
with open('data_with_passivity.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Read the YAML variables
time = np.array(data['time'])
V1_values = np.array(data['V1_values'])
xG_values = np.array(data['xG_values'])
pG_values = np.array(data['pG_values'])
velocity_values = np.array(data['velocity_values'])
velocity_human_values = np.array(data['velocity_human_values'])
u_i_values = np.array(data['u_i_values'])
del_uh_i_values = np.array(data['del_uh_i_values'])
uH_values = np.array(data['uH_values'])
HI_values = np.array(data['HI_values'])
x2_i_values = np.array(data['x2_i_values'])


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
plt.show()

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
plt.show()

##############################################################################################################################
################################################# Plot Passivity Graph #######################################################
##############################################################################################################################

#plot the graph for passivity
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

plt.figure(figsize=(10, 6))

plt.plot(time, velocity_values, c="red", label='$x_{i,2}^T \cdot u_i$')
plt.plot(time, velocity_human_values, c="blue", label='$u_H^T \cdot \Delta u_H$')

#plt.ylim(-0.02, 0.02)
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('passivity.png', dpi=1200)
plt.show()

##############################################################################################################################
################################################# Plot u vs time Graph #######################################################
##############################################################################################################################

#plot the graph of u vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))
for i in range(0,8,2):
    plt.plot(time, u_i_values[i], c=colors[i], label='$u_{%d,%d}$' % (int(i/2)+1,1))
    plt.plot(time, u_i_values[i+1], c=colors[i+1], label='$u_{%d,%d}$' % (int(i/2)+1,2))

plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right', ncol = 4)
plt.grid(True)

#plt.savefig('u_values.png', dpi=1200)
plt.show()

##############################################################################################################################
################################################# Plot del_uh vs time Graph ##################################################
##############################################################################################################################

#plot the graph of del_uh vs time
plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

plt.figure(figsize=(10, 6))

plt.plot(time, del_uh_i_values[0], c=colors[6], label='$\Delta u_{H_{1}}$')
plt.plot(time, del_uh_i_values[1], c=colors[7], label='$\Delta u_{H_{2}}$')

plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right', ncol = 1)
plt.grid(True)

#plt.savefig('del_uh_values.png', dpi=1200)
plt.show()

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
plt.show()

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
plt.legend(loc = 'upper right', ncol = 4)
plt.grid(True)

#plt.savefig('x2_values.png', dpi=1200)
plt.show()