import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from state_space_2DOF import *
from luenberger_2DOF import *

#ISSUES:
'''
    1) Fix issue with control: the first link moves slightly when the top link moves but not vice versa
'''

'''
    @GLOBAL PARAMS:
        Link length (m) - lLength
        Link mass (kg) - lMass
        gravity acceleration (m/s^2) - g
        center of mass for links - cMass
        moment of Inertia for links - inertia
        Process noise covariance matrix - Q
        Measurement noise covariance matrix - R
'''
lLengths = [1.0, 1.0]
lMass = [1.0, 1.0]
grav = 9.81
cMass = [lLengths[0]/2, lLengths[1]/2]
inertia = [(1 / 3) * mass * length ** 2 for mass, length in zip(lMass, lLengths)]
(stateMatrix, inputMatrix,
    outputMatrix, feedthroughMatrix) = generate_SS(lLengths, lMass)
Q = np.eye(stateMatrix.shape[0]) * 0.1  # Process noise covariance matrix
R = np.eye(outputMatrix.shape[0]) * 0.1
stop_simulation = False


'''
    @Initial Setup:
        initial angles (rad) - theta
        initial angular velocities (rad/sec) - dtheta
        time step (s) - dt
        Total simulation runtime (s) - T
        initial state - state_estimate
        control placeholder for Observer - u
'''
observer_gain = calculate_observer_gain(stateMatrix, outputMatrix, Q, R)
theta = np.array([np.pi / 2, np.pi / 2])
dtheta = np.array([0.0, 0.0])
dt = 0.01
T = 10.0
n_steps = int(T / dt)
state = np.hstack((theta, dtheta))
state_estimate = np.copy(state)
previous_state_estimate = None
u = np.zeros(len(lLengths))


'''
    @key_press: converts keyboard input into movements for 2d arm

    @params: none
    @returns: none
'''
def key_press(event):
    global state, theta, dtheta, u, state_estimate, previous_state_estimate
    new_theta = theta.copy()
    increment = 0.05
    key_map = {'left': 0, 'right': 0, 'a': 1, 'd': 1}

    if event.key in key_map:
        index = key_map[event.key]
        if event.key == 'left' or event.key == 'a':
            new_theta[index] += increment
        elif event.key == 'right' or event.key == 'd':
            new_theta[index] -= increment

        link1_y = lLengths[0] * np.sin(new_theta[0])
        link2_y = link1_y + lLengths[1] * np.sin(new_theta[0] + new_theta[1])
        if link1_y >= 0 and link2_y >= 0:
            theta[:] = new_theta
            state = np.hstack((theta, dtheta))
            state = update_SS(state, stateMatrix, inputMatrix, dtheta, dt)
            y = outputMatrix @ state
            previous_state_estimate = np.copy(state_estimate)  # Store the previous state estimate
            state_estimate = update_observer(state_estimate, stateMatrix, inputMatrix, outputMatrix, observer_gain, u, y, dt)
            update_robot(state, state_estimate)

'''
    @stop_simulation: stops the simulation so it doesnt like break frfr
'''
def stop_simulation(event):
    global stop_simulation
    stop_simulation = True

'''
    @update_robot: updates the position of the links in the simulation

    @params: state - current state model of robot
    @params: state_estimate - predicted state estimate of the robot
    
    @returns: none
'''
def update_robot(state, state_estimate):
    global theta, theta_history, previous_state_estimate, difference_var
    theta = state[:2]
    theta_history.append(np.copy(theta))
    ax.clear()

    # Plot update
    x = [0, lLengths[0] * np.cos(theta[0]), lLengths[0] * np.cos(theta[0]) + lLengths[1] * np.cos(theta[0] + theta[1])]
    y = [0, lLengths[0] * np.sin(theta[0]), lLengths[0] * np.sin(theta[0]) + lLengths[1] * np.sin(theta[0] + theta[1])]
    ax.plot(x, y, '-o', linewidth=2)
    ax.plot([-2, 2], [0, 0], 'k--')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    canvas.draw()

    # Calculate and display the difference between predicted and actual state
    if previous_state_estimate is not None:
        difference = state - previous_state_estimate
    else:
        difference = np.zeros_like(state)

    # Update info window
    joint_vars = [joint1_var, joint2_var]
    for i, var in enumerate(joint_vars):
        var.set(f"Joint {i + 1}: {np.degrees(theta[i]):.5f}°")
    state_str = np.array2string(state, formatter={'float_kind': lambda x: "%.5f" % x}, separator=',', suppress_small=True)
    state_var.set(f"State: {state_str}")
    state_estimate_str = np.array2string(state_estimate, formatter={'float_kind': lambda x: "%.5f" % x}, separator=',', suppress_small=True)
    state_estimate_var.set(f"Estimated State: {state_estimate_str}")
    difference_str = np.array2string(difference, formatter={'float_kind': lambda x: "%.5f" % x}, separator=',', suppress_small=True)
    difference_var.set(f"Difference: {difference_str}")
    state_matrix_var.set(f"stateMatrix: {np.array2string(stateMatrix, precision=2, separator=',')}")
    input_matrix_var.set(f"inputMatrix: {np.array2string(inputMatrix, precision=2, separator=',')}")
    output_matrix_var.set(f"outputMatrix: {np.array2string(outputMatrix, precision=2, separator=',')}")
    feedthrough_matrix_var.set(f"feedthroughMatrix: {np.array2string(feedthroughMatrix, precision=2, separator=',')}")


'''
    @create_info(): creates the information window for the robot simulation
'''
def create_info():
    info_window = tk.Toplevel(root)
    info_window.title("2DoF Arm - Force Estimation")

    # Create vars
    global joint1_var, joint2_var, state_var, state_estimate_var, difference_var, state_matrix_var, input_matrix_var, output_matrix_var, feedthrough_matrix_var
    joint1_var = tk.StringVar()
    joint2_var = tk.StringVar()
    state_var = tk.StringVar()
    state_estimate_var = tk.StringVar()
    difference_var = tk.StringVar()
    state_matrix_var = tk.StringVar()
    input_matrix_var = tk.StringVar()
    output_matrix_var = tk.StringVar()
    feedthrough_matrix_var = tk.StringVar()

    # Create labels for state-space matrices
    state_matrix_label = tk.Label(info_window, textvariable=state_matrix_var)
    input_matrix_label = tk.Label(info_window, textvariable=input_matrix_var)
    output_matrix_label = tk.Label(info_window, textvariable=output_matrix_var)
    feedthrough_matrix_label = tk.Label(info_window, textvariable=feedthrough_matrix_var)
    state_matrix_label.grid(row=0, column=0, padx=10, pady=5)
    input_matrix_label.grid(row=0, column=1, padx=10, pady=5)
    output_matrix_label.grid(row=0, column=2, padx=10, pady=5)
    feedthrough_matrix_label.grid(row=0, column=3, padx=10, pady=5)

    # Create labels for joint angles, state, state estimate, and difference
    joint1_label = tk.Label(info_window, textvariable=joint1_var)
    joint2_label = tk.Label(info_window, textvariable=joint2_var)
    state_label = tk.Label(info_window, textvariable=state_var)
    state_estimate_label = tk.Label(info_window, textvariable=state_estimate_var)
    difference_label = tk.Label(info_window, textvariable=difference_var)
    joint1_label.grid(row=1, column=0, padx=10, pady=5)
    joint2_label.grid(row=1, column=1, padx=10, pady=5)
    state_label.grid(row=1, column=2, padx=10, pady=5)
    state_estimate_label.grid(row=1, column=3, padx=10, pady=5)
    difference_label.grid(row=1, column=4, padx=10, pady=5)


'''
    @make_tkinter: Creates the tkinter window for modeling 2dof robot and information
    
    @params: none
    @return: none
'''
def make_tkinter():
    global root, frame, canvas, fig, ax
    root = tk.Tk()
    root.title("2Dof Arm - Force Estimation")

    # Main frame for canvas
    frame = tk.Frame(root)
    frame.grid(row=0, column=0, columnspan=4, sticky="nsew")
    fig, ax = plt.subplots()

    # Create canvas
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    fig.canvas.mpl_connect('key_press_event', key_press)
    root.bind("<Escape>", stop_simulation)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.grid_columnconfigure(2, weight=1)
    root.grid_columnconfigure(3, weight=1)
    create_info()

theta_history = []

def main():
    make_tkinter()
    update_robot(state, state_estimate)
    root.mainloop()

if __name__ == "__main__":
    main()
