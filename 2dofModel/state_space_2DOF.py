import numpy as np

#GLOBAL
grav = 9.81

'''
    @generate_SS: generates the state space model for the robotic arm

    @param: link_length -> Vector of link lengts for each on in the robot
    @param: link_mass -> Vector of link masses for each on in the robot

    @returns: SS_model: state space model where each element is a state space
                        matrix
'''
def generate_SS(link_length, link_mass):
    
    #make robot params
    linkNum = len(link_length)
    link_lengths = np.array(link_length)
    link_masses = np.array(link_mass)
    cMass = link_lengths / 2 # comeback to fix
    inertia = np.array((1/3) * link_masses * link_lengths ** 2)

    #State space matricies
    stateMatrix = np.zeros((2*linkNum, 2*linkNum))
    inputMatrix = np.zeros((2*linkNum, linkNum))
    outputMatrix = np.eye(2*linkNum)
    feedthroughMatrix = np.zeros((2*linkNum, linkNum))

    for i in range(linkNum):
        # Define state dynamics elements
        if i < linkNum - 1:
            stateMatrix[2 * i, 2 * i + 1] = 1
            stateMatrix[2 * i + 1, 2 * i + 2] = (-link_masses[i + 1] * grav * cMass[i + 1]
                                                 / (inertia[i] + inertia[i + 1]))
            stateMatrix[2 * i + 2, 2 * i + 3] = 1
            stateMatrix[2 * i + 3, 2 * i] = ((link_masses[i] + link_masses[i + 1]) *
                                             grav * cMass[i] / (inertia[i] + inertia[i + 1]))

        # Define B matrix elements for control inputs
        inputMatrix[2 * i + 1, i] = 1 / inertia[i]
        if i < linkNum - 1:
            inputMatrix[2 * i + 3, i + 1] = 1 / inertia[i + 1]

    return [stateMatrix, inputMatrix, outputMatrix, feedthroughMatrix]

'''
    @update_SS: updates state vector w/ state space model

    @params: state -> cur state vector
    @params: stateMatrix -> state matrix
    @params: inputMatrix -> input matrix
    @params: t -> input vector/torques

    @returns: updateState -> updates state vector
    
    xDot = Ax + Bt
    state_new = A.dot(x) + b.dot(dtheta)
'''
def update_SS(state, stateMatrix, inputMatrix, t, dt):
    x_dot = stateMatrix @ state + inputMatrix @ t
    return state + dt * x_dot
