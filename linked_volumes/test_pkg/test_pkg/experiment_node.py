import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
# from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray
import numpy as np

##############################################################################################################################
###################################################### Libraries #############################################################
##############################################################################################################################

import matplotlib.pyplot as plt
from cvxopt import solvers, matrix
import matplotlib.animation as animation
from .operations.mobile_manipulator import MobileManipulator

##############################################################################################################################
###################################################### Variable Initialization ###############################################
##############################################################################################################################

# # Simulation parameters
# DT = 0.02
# t_end = 100



##############################################################################################################################
######################################## Optimization Algorithm (Quadprog) (With Passivity) ##################################
##############################################################################################################################

# while t <= t_end:
#   pass


# ##############################################################################################################################
# ###################################################### Plot Formation Graph ##################################################
# ##############################################################################################################################

# #plot the graph for formation
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

# plt.figure(figsize=(10, 6))

# plt.plot(time, V1_values, c="purple", label=f'V1')
    

# plt.xlabel('time (s)')
# plt.ylabel('V1 (m/s)')
# plt.legend(loc = 'upper right')
# plt.grid(True)

# #plt.savefig('xG_and_pG.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ################################################# Plot xG, pG vs time Graph ##################################################
# ##############################################################################################################################

# #plot the graph for xG, pG vs time
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

# plt.figure(figsize=(10, 6))

# plt.plot(time, xG_values[0], c="red", label=f'xG1')
# plt.plot(time, xG_values[1], c="green", label=f'xG2')
# plt.plot(time, pG_values[0], c="blue", label=f'pG1')
# plt.plot(time, pG_values[1], c="fuchsia", label=f'pG2')
    

# plt.xlabel('time (s)')
# plt.ylabel('xG and pG (m)')
# plt.legend(loc = 'upper right')
# plt.grid(True)

# #plt.savefig('formation_graph.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ################################################# Plot Passivity Graph #######################################################
# ##############################################################################################################################

# #plot the graph for passivity
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

# plt.figure(figsize=(10, 6))

# plt.plot(time, velocity_values, c="red", label=f'velocity')
# plt.plot(time, velocity_human_values, c="blue", label=f'velocity_human')

# plt.ylim(-0.02, 0.02)
# plt.xlabel('time (s)')
# plt.ylabel('speed (m/s)')
# plt.legend(loc = 'upper right')
# plt.grid(True)

# #plt.savefig('passivity.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ################################################# Plot u vs time Graph #######################################################
# ##############################################################################################################################

# #plot the graph of u vs time
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
# colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

# plt.figure(figsize=(10, 6))
# for i in range(0,8,2):
#     plt.plot(time, u_i_values[i], c=colors[i], label=f'u{int(i/2)+1}1')
#     plt.plot(time, u_i_values[i+1], c=colors[i+1], label=f'u{int(i/2)+1}2')

# plt.xlabel('time (s)')
# plt.ylabel('speed (m/s)')
# plt.legend(loc = 'upper right', ncols = 4)
# plt.grid(True)

# #plt.savefig('u_values.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ################################################# Plot del_uh vs time Graph ##################################################
# ##############################################################################################################################

# #plot the graph of del_uh vs time
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
# colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

# plt.figure(figsize=(10, 6))

# plt.plot(time, del_uh_i_values[0], c=colors[6], label='del_uh1')
# plt.plot(time, del_uh_i_values[1], c=colors[7], label='del_uh2')

# plt.xlabel('time (s)')
# plt.ylabel('speed (m/s)')
# plt.legend(loc = 'upper right', ncols = 1)
# plt.grid(True)

# #plt.savefig('del_uh_values.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ########################################## Plot uH, uH+del_uH vs time Graph ##################################################
# ##############################################################################################################################

# #plot the graph for uH, uH+del_uH vs time
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background

# plt.figure(figsize=(10, 6))

# plt.plot(time, uH_values[0], c="red", label=f'uH1')
# plt.plot(time, uH_values[1], c="blue", label=f'uH2')
# plt.plot(time, HI_values[0], c="green", label=f'uH1+del_uH1')
# plt.plot(time, HI_values[1], c="brown", label=f'uH2+del_uH2')

# plt.xlabel('time (s)')
# plt.ylabel('uH and uH+del_uH (m/s)')
# plt.legend(loc = 'upper right')
# plt.grid(True)

# #plt.savefig('uH and uH+del_uH.png', dpi=1200)
# #plt.show()

# ##############################################################################################################################
# ################################################# Plot x2 vs time Graph ######################################################
# ##############################################################################################################################

# #plot the graph of x2 vs time
# plt.style.use('default')    # 'default' for white background, 'dark_background' for black background
# colors = np.array(["red","blue","green","brown","yellow","orange","fuchsia","darkviolet"])

# plt.figure(figsize=(10, 6))

# for i in range(0,8,2):
#   plt.plot(time, x2_i_values[i], c=colors[i], label=f'x{int(i/2)+1}21')
#   plt.plot(time, x2_i_values[i+1], c=colors[i+1], label=f'x{int(i/2)+1}22')

# plt.xlabel('time (s)')
# plt.ylabel('x2 (speed)(m/s)')
# plt.legend(loc = 'upper right', ncols = 4)
# plt.grid(True)

# #plt.savefig('x2_values.png', dpi=1200)
# plt.show()













# num_robot = 4

class MyNode(Node):
    def __init__(self):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('experiment_node')

        broker_ip = "192.168.0.2"
        self.robots = [
            MobileManipulator(group_id="2", backend_server_ip=broker_ip),
            MobileManipulator(group_id="5", backend_server_ip=broker_ip),
            MobileManipulator(group_id="6", backend_server_ip=broker_ip),
            MobileManipulator(group_id="8", backend_server_ip=broker_ip)
        ]

        # create the publisher object
        self.publisher_haptic = self.create_publisher(Float64MultiArray, '/fd/fd_controller/commands', 1)
        
        # create the subscriber object
        self.ee_pose_subscription = self.create_subscription(PoseStamped, '/fd/ee_pose', self.pose_callback, 1)

        # define the variable to save the received info
        self.feedback = Float64MultiArray()

        # Constant parameters
        self.DT = 0.02
        self.k = 20.  #kappa
        self.kh = 1.
        self.kv = 0.1
        self.kp = 0.1

        self.t = 0.0
        
        start_position = np.array([[1.5, 0.5, 1.5, 0.5],
                                [-1.5, -1.5, -0.5, -0.5]])
        vel = np.array([[0., 0., 0., 0.],
                        [0., 0., 0., 0.]])

        # Initialize robot position and velocity
        self.states = np.append(start_position, vel, axis=0)

        self.haptic_position = np.array([0.0, 0.0])


        self.gama = np.array([0.3,0.1])
        self.alpha1 = 2.

        self.Aa = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
        self.Bb = np.block([[np.zeros((2, 2))], [np.eye(2)]])

        self.pG = np.array([[-0.75], [0.75]])
        
        self.neighbour = np.array([[0, 0.8, 0.8, 1.13],
                            [0.8, 0, 1.13, 0.8],
                            [0.8, 1.13, 0, 0.8],
                            [1.13, 0.8, 0.8, 0]])

        # Create lists to store robot states for plotting
        self.robot_history = self.states

        self.time = np.array([])
        self.V1_values = np.array([])
        self.xG_values = np.array([[],[]])
        self.pG_values = np.array([[],[]])
        self.velocity_values = np.array([])
        self.velocity_human_values = np.array([])
        self.u_i_values = np.array([[], [], [], [], [], [], [], []])
        self.del_uh_i_values = np.array([[], []])
        self.uH_values = np.array([[], []])
        self.HI_values = np.array([[], []])
        self.x2_i_values = np.array([[], [], [], [], [], [], [], []])

        self.H = np.block([[np.eye(8), np.zeros((8, 4)), np.zeros((8, 2))], [np.zeros((4, 8)), self.k * np.eye(4), np.zeros((4, 2))], [np.zeros((2, 8)), np.zeros((2, 4)), self.kh * np.eye(2)]])
        self.f = np.zeros((14,1))

        # Animation Code 
        # Define the points of the fixed quadrilateral
        self.quad_points = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])

        self.colors = np.array(["red","blue","green","brown"])

        # Create the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.point1, = self.ax.plot([], [], 'o', color=self.colors[0], label='Robot 1')
        self.point2, = self.ax.plot([], [], 'o', color=self.colors[1], label='Robot 2')
        self.point3, = self.ax.plot([], [], 'o', color=self.colors[2], label='Robot 3')
        self.point4, = self.ax.plot([], [], 'o', color=self.colors[3], label='Robot 4')
        self.point5, = self.ax.plot([], [], 'x', color="purple", label='xG')
        self.point6, = self.ax.plot([], [], 'x', color=self.colors[3], label='pG')

        # Plot the fixed quadrilateral
        self.quad = plt.Polygon(self.quad_points, closed=True, edgecolor='C9', facecolor='none', label='Fixed Quadrilateral')

        # Set the axis limits
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.grid(True)  # Add grid lines

        # Add legend
        self.ax.legend()

        # Add the quadrilateral to the plot
        self.ax.add_patch(self.quad)

        # Add labels and title
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Robot Motion')

        # Set the aspect ratio to be equal
        self.ax.set_aspect('equal')

        self.timer = self.create_timer(0.02, self.motion)

    def get_poses(self):
        self.poses = [[], [], [], []]
        for i in range(4):
            qi = self.robots[i].get_poses()
            if qi:
                self.poses[i] = qi
    
    def send_velocities(self, v):
        # v : (NUM_ROBOTS x 2)-array
        for i in range(4):
            omega_i = -10.0 * self.poses[i][2]
            self.robots[i].set_mobile_base_speed_and_gripper_power(v[i, 0], v[i, 1], omega_i, 0.)
    
    def pose_callback(self, msg):
        # Read the linear x and y positions from the PoseStamped message
        self.haptic_position[0] = msg.pose.position.y
        self.haptic_position[1] = -msg.pose.position.x
        self.get_logger().info("x: {}       y: {}".format(self.haptic_position[0],self.haptic_position[1]))

    def motion(self):
        # robots: get poses ####################################################################
        self.get_poses()
        print('robots: get poses: ', self.poses)
        if all(self.poses):
            for i in range(4):
                self.states[0:2,i] = self.poses[i][0:2]
        else:
            return
        ########################################################################################

        xG = np.zeros((2,1))
        vG = np.zeros((2,1))
        for i in range(4):
            xG = xG + self.states[0:2,i].reshape(-1,1)
            vG = vG + self.states[2:4,i].reshape(-1,1)

        xG = (1/4) * xG
        
        # uH = self.kp * (self.pG - xG)
        # haptics: position command #############################################################
        uH = 2.0 * self.haptic_position.reshape(-1,1)
        uH_bound = 0.1
        uH[0] = min(uH_bound,max(-uH_bound,uH[0]))
        uH[1] = min(uH_bound,max(-uH_bound,uH[1]))
        print('haptics: commanded velocity: ', uH.T)
        ########################################################################################

        vG = (1/4) * vG
        
        
        Jx = np.zeros((4,))
        for i in range(4):
            count1 = 0
            for j in range(4):
                if self.neighbour[i][j] != 0:
                    count1 = count1 + 0.5 * (np.linalg.norm(self.states[0:2,i] - self.states[0:2,j]) - self.neighbour[i][j]) ** 2
            Jx[i] = count1

        Lx = np.zeros((4,))
        for i in range(4):
            Lx[i] = 0.5 * self.alpha1 * (np.linalg.norm(self.states[2:4,i])) **2

        V1 = Jx + Lx

        dJxi = np.zeros((4,2))
        for i in range(4):
            temp2 = np.zeros((1,2))
            for j in range(4):
                if self.neighbour[i][j] != 0:
                    temp1 = np.linalg.norm(self.states[0:2,i] - self.states[0:2,j])
                    temp2 = temp2 + ((temp1 - self.neighbour[i][j]) / temp1) * (self.states[0:2,i] - self.states[0:2,j]).T
            dJxi[i] = 2 * temp2

        dV1 = np.block([dJxi, self.alpha1 * ((self.states[2:4]).T)])

        zeros = np.zeros((1,2))
        conv6 = -(uH.T)/4
        eye = (1/4) * np.eye(2)
        A1 = np.block([[dV1[0] @ self.Bb, zeros, zeros, zeros], [zeros, dV1[1] @ self.Bb, zeros, zeros], [zeros, zeros, dV1[2] @ self.Bb, zeros], [zeros, zeros, zeros, dV1[3] @ self.Bb]])
        A2 = np.block([-1 * np.eye(4)])
        A3 = np.block([[zeros], [zeros], [zeros], [zeros]])
        A4 = np.block([[(self.states[2:4,0]).T, zeros, zeros, zeros], [zeros, (self.states[2:4,1]).T, zeros, zeros], [zeros, zeros, (self.states[2:4,2]).T, zeros], [zeros, zeros, zeros, (self.states[2:4,3]).T]])
        A5 = np.zeros((4,4))
        A6 = np.block([[conv6], [conv6], [conv6], [conv6]])

        A = np.block([[A1, A2, A3], [A4, A5, A6]])
        b = np.array([-(self.gama[0] * V1[0]) - (dV1[0] @ self.Aa @ self.states[:,0]), -(self.gama[0] * V1[1]) - (dV1[1] @ self.Aa @ self.states[:,1]), -(self.gama[0] * V1[2]) - (dV1[2] @ self.Aa @ self.states[:,2]), -(self.gama[0] * V1[3]) - (dV1[3] @ self.Aa @ self.states[:,3]), 0, 0, 0, 0])
        b = b.reshape(-1,1)
        C = np.block([eye, eye, eye, eye, np.zeros((2, 4)), -self.kv * np.eye(2)])
        d = self.kv * uH - self.kv * vG
        d = d.reshape(-1,1)

        # Solve quadratic programming problems using cvxopt.solve_qp for u
        solvers.options['show_progress'] = False  # Hide progress output
        v = solvers.qp(matrix(self.H), matrix(self.f), matrix(A), matrix(b), matrix(C), matrix(d))

        u = np.array(v['x'])[0:8]

        delt = np.array(v['x'])[8:12]

        del_uh = np.array(v['x'])[12:14]

        #temp_vec4 = np.array(u4['x'])[4:6]
        #vec4 = np.append(vec4, temp_vec4, axis=1)

        velocity = self.states[2:4,:].T.flatten() @ u
        velocity_human = (uH.T) @ del_uh
        HI = uH + del_uh

        for i in range(4):
            self.states[:,i] = self.states[:,i] + (self.Aa @ self.states[:,i] + self.Bb @ u[(i*2):((i*2)+2),0]) * self.DT
        
        # robots: send velocities ##############################################################
        self.send_velocities(self.states[2:4,:].T)
        print('robots: send velocities: ', self.states[2:4,:])
        ########################################################################################

        self.robot_history = np.dstack((self.robot_history,self.states))


        temp_u_i_values = u
        temp_del_uh_i_values = del_uh
        temp_x2_i_values = self.states[2:4,:].T.flatten()
        temp_x2_i_values = temp_x2_i_values.reshape(-1,1)
        #print(temp_x2_i_values)
        
        self.time = np.append(self.time,self.t)
        self.V1_values = np.append(self.V1_values, V1[0]+V1[1]+V1[2]+V1[3])
        self.xG_values = np.append(self.xG_values, xG, axis = 1)
        self.pG_values = np.append(self.pG_values, self.pG, axis = 1)
        self.velocity_values = np.append(self.velocity_values, velocity)
        self.velocity_human_values = np.append(self.velocity_human_values, velocity_human)
        self.u_i_values = np.append(self.u_i_values, temp_u_i_values, axis = 1)
        self.del_uh_i_values = np.append(self.del_uh_i_values, temp_del_uh_i_values, axis = 1)
        self.uH_values = np.append(self.uH_values, uH, axis = 1)
        self.HI_values = np.append(self.HI_values, HI, axis = 1)
        self.x2_i_values = np.append(self.x2_i_values, temp_x2_i_values, axis = 1)
            
        self.point1.set_data([self.robot_history[0, 0, -1]], [self.robot_history[1, 0, -1]])
        self.point2.set_data([self.robot_history[0, 1, -1]], [self.robot_history[1, 1, -1]])
        self.point3.set_data([self.robot_history[0, 2, -1]], [self.robot_history[1, 2, -1]])
        self.point4.set_data([self.robot_history[0, 3, -1]], [self.robot_history[1, 3, -1]])
        self.point5.set_data([self.xG_values[0,-1]], [self.xG_values[1,-1]])
        self.point6.set_data([self.pG[0,-1]], [self.pG[1,-1]])
        
        plt.draw()
        plt.pause(0.0001)

        # haptics: force feedback ###############################################################
        force = 10.0 * del_uh
        self.feedback.data = [force[0,0], force[1,0], 0.0]
        print('haptics: feedback force: ', force.T)
        self.publisher_haptic.publish(self.feedback)
        ########################################################################################

        self.t = self.t + self.DT
        print(self.t)
        save_condition = abs(self.t - 10.0) <= 10 * self.DT or abs(self.t - 20.0) <= 10 * self.DT or abs(self.t - 30.0) <= 10 * self.DT or abs(self.t - 40.0) <= 10 * self.DT or abs(self.t - 50.0) <= 10 * self.DT or abs(self.t - 60.0) <= 10 * self.DT
        if save_condition:
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_time.csv", self.time, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_V1.csv", self.V1_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_xG.csv", self.xG_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_pG.csv", self.pG_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_vel.csv", self.velocity_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_vel_human.csv", self.velocity_human_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_u.csv", self.u_i_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_del_uh.csv", self.del_uh_i_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_uH.csv", self.uH_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_HI.csv", self.HI_values, delimiter=",")
            np.savetxt("/ros2_ws/src/test_pkg/data/exp_data_x2.csv", self.x2_i_values, delimiter=",")

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = MyNode()
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # Explicity destroy the node
    node.destroy_node()
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
