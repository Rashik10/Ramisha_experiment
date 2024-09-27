import yaml
import numpy as np
import matplotlib.pyplot as plt

# Load the YAML file
with open('data_without_passivity.yaml', 'r') as file:
    data = yaml.safe_load(file)

# Read the YAML variables
time = np.array(data['time'])
V1_values = np.array(data['V1_values'])
xG_values = np.array(data['xG_values'])
velocity_values = np.array(data['velocity_values'])
velocity_human_values = np.array(data['velocity_human_values'])


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

plt.plot(time, velocity_values, c="red", label='$x_{i,2}^T \cdot u_i$')
plt.plot(time, velocity_human_values, c="blue", label='$u_H^T \cdot \Delta u_H$')

#plt.ylim(-0.1, 0.1)
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')
plt.legend(loc = 'upper right')
plt.grid(True)

#plt.savefig('u_values.png', dpi=1200)
plt.show()