import numpy as np
from cvxopt import solvers, matrix
import yaml

def cent_with_passivity(states_old, haptic):   
    # Simulation parameters
    DT = 0.02

    # Constant parameters
    k = 35.  #kappa
    kh = 1.
    kv = 0.1
    kp = 0.1
    #K = 100

    gama = np.array([0.3,0.1])
    alpha1 = 2.

    Aa = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    Bb = np.block([[np.zeros((2, 2))], [np.eye(2)]])

    pG= np.array([[-0.75], [0.75]])

    neighbour = np.array([[0, 0.8, 0.8, 1.13],
                        [0.8, 0, 1.13, 0.8],
                        [0.8, 1.13, 0, 0.8],
                        [1.13, 0.8, 0.8, 0]])

    
    H = np.block([[np.eye(8), np.zeros((8, 4)), np.zeros((8, 2))], [np.zeros((4, 8)), k * np.eye(4), np.zeros((4, 2))], [np.zeros((2, 8)), np.zeros((2, 4)), kh * np.eye(2)]])
    f = np.zeros((14,1))

    states = states_old
    temp_states = np.empty(states.shape)
    xG = np.zeros((2,1))
    vG = np.zeros((2,1))
    for i in range(4):
        xG = xG + states[0:2,i].reshape(-1,1)
        vG = vG + states[2:4,i].reshape(-1,1)

    xG = (1/4) * xG
    uH = haptic.reshape(-1,1)
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

    velocity = states[2:4,:].T.flatten() @ u
    velocity_human = (uH.T) @ del_uh
    HI = uH + del_uh

    for i in range(4):
        temp_states[:,i] = (Aa @ states[:,i] + Bb @ u[(i*2):((i*2)+2),0]) * DT

    vel = temp_states[2:4]

    temp_u_i_values = u
    temp_del_uh_i_values = del_uh
    temp_x2_i_values = states[2:4,:].T.flatten()
    temp_x2_i_values = temp_x2_i_values.reshape(-1,1)


    # Load the array from the YAML file
    with open('/home/ramisha/ros2_ws/src/test_pkg/test_pkg/operations/data_with_passivity.yaml', 'r') as file:
        data = yaml.safe_load(file)
        
    t = data['t']
    
    if(t != 0):
        # Read the values from YAML file
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

        t = t + 1
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
        t = t + 1
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
    

    # Write the updated array back to the YAML file
    data['t'] = t
    data['time'] = time.tolist()
    data['V1_values'] = V1_values.tolist()
    data['xG_values'] = xG_values.tolist()
    data['pG_values'] = pG_values.tolist()
    data['velocity_values'] = velocity_values.tolist()
    data['velocity_human_values'] = velocity_human_values.tolist()
    data['u_i_values'] = u_i_values.tolist()
    data['del_uh_i_values'] = del_uh_i_values.tolist()
    data['uH_values'] = uH_values.tolist()
    data['HI_values'] = HI_values.tolist()
    data['x2_i_values'] = x2_i_values.tolist()


    with open('/home/ramisha/ros2_ws/src/test_pkg/test_pkg/operations/data_with_passivity.yaml', 'w') as file:
        yaml.dump(data, file, default_flow_style=False)   

    return vel, del_uh




def cent_without_passivity(states_old, haptic):   
    # Simulation parameters
    DT = 0.02

    # Constant parameters
    k = 35.  #kappa
    kh = 1.
    kv = 0.1
    #kp = 0.1
    #K = 100

    gama = np.array([0.3,0.1])
    alpha1 = 2.

    Aa = np.block([[np.zeros((2, 2)), np.eye(2)], [np.zeros((2, 2)), np.zeros((2, 2))]])
    Bb = np.block([[np.zeros((2, 2))], [np.eye(2)]])

    pG= np.array([[-0.75], [0.75]])

    neighbour = np.array([[0, 0.8, 0.8, 1.13],
                        [0.8, 0, 1.13, 0.8],
                        [0.8, 1.13, 0, 0.8],
                        [1.13, 0.8, 0.8, 0]])

    
    H = np.block([[np.eye(8), np.zeros((8, 4)), np.zeros((8, 2))], [np.zeros((4, 8)), k * np.eye(4), np.zeros((4, 2))], [np.zeros((2, 8)), np.zeros((2, 4)), kh * np.eye(2)]])
    f = np.zeros((14,1))
    
    states = states_old
    temp_states = np.empty(states.shape)
    xG = np.zeros((2,1))
    vG = np.zeros((2,1))
    for i in range(4):
        xG = xG + states[0:2,i].reshape(-1,1)
        vG = vG + states[2:4,i].reshape(-1,1)

    xG = (1/4) * xG
    uH = haptic.reshape(-1,1)
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

    A = np.block([A1, A2, A3])
    b = np.array([-(gama[0] * V1[0]) - (dV1[0] @ Aa @ states[:,0]), -(gama[0] * V1[1]) - (dV1[1] @ Aa @ states[:,1]), -(gama[0] * V1[2]) - (dV1[2] @ Aa @ states[:,2]), -(gama[0] * V1[3]) - (dV1[3] @ Aa @ states[:,3])])
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

    velocity = states[2:4,:].T.flatten() @ u
    velocity_human = (uH.T) @ del_uh

    for i in range(4):
        temp_states[:,i] = (Aa @ states[:,i] + Bb @ u[(i*2):((i*2)+2),0]) * DT
    
    vel = states[2:4]

    # Load the array from the YAML file
    with open('/home/ramisha/ros2_ws/src/test_pkg/test_pkg/operations/data_without_passivity.yaml', 'r') as file:
        data = yaml.safe_load(file)
        
    t = data['t']

    if(t != 0):
        # Read the values from YAML file
        time = np.array(data['time'])
        V1_values = np.array(data['V1_values'])
        xG_values = np.array(data['xG_values'])
        velocity_values = np.array(data['velocity_values'])
        velocity_human_values = np.array(data['velocity_human_values'])

        t = t + 1        
        time = np.append(time,t)
        V1_values = np.append(V1_values, V1[0]+V1[1]+V1[2]+V1[3])
        xG_values = np.append(xG_values, xG, axis = 1)
        velocity_values = np.append(velocity_values, velocity)
        velocity_human_values = np.append(velocity_human_values, velocity_human)
    else:
        t = t + 1
        time = np.array([t])
        V1_values = np.array(V1[0]+V1[1]+V1[2]+V1[3])
        xG_values = xG
        velocity_values = velocity
        velocity_human_values = velocity_human

    # Write the updated array back to the YAML file
    data['t'] = t
    data['time'] = time.tolist()
    data['V1_values'] = V1_values.tolist()
    data['xG_values'] = xG_values.tolist()
    data['velocity_values'] = velocity_values.tolist()
    data['velocity_human_values'] = velocity_human_values.tolist()

    with open('/home/ramisha/ros2_ws/src/test_pkg/test_pkg/operations/data_without_passivity.yaml', 'w') as file:
        yaml.dump(data, file, default_flow_style=False)   

    return vel, del_uh