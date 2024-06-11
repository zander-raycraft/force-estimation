import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from state_space_2DOF import *

'''
    @GLOBAL PARAMS:
        Link length (m) - lLength
        Link mass (kg) - lMass
        gravity acceleration (m/s^2) - g
        center of mass for links - cMass
        moment of Inertia for links - inertia
'''
lLengths = [1.0, 1.0]
lMass = [1.0, 1.0]
grav = 9.81
cMass = [lLengths[0]/2, lLengths[1]/2]
inertia = [((1/3) * lMass[0] * lLengths[0] ** 2), ((1/3) * lMass[1] * lLengths[1] ** 2)]
(stateMatrix, inputMatrix,
    outputMatrix, feedthroughMatrix) = generate_SS(lLengths, lMass)

'''
    @Initial Setup:
        initial angles (rad) - theta
        initial angular velocities (rad/sec) - dtheta
        time step (s) - dt
        Total simulation runtime (s) - T
'''
theta = np.array([np.pi / 2, np.pi / 2])
dtheta = np.array([0.0, 0.0])
dt = 0.01
T = 10.0
n_steps = int(T / dt)
state = np.hstack((theta, dtheta))


'''
    @key_press: converts keyboard input into movements for 2d arm

    @params: none
    @returns: none
'''
def key_press(event):
    global state, theta, dtheta
    new_theta = theta.copy()

    if event.key == 'left':
        new_theta[0] += 0.05
    elif event.key == 'right':
        new_theta[0] -= 0.05
    elif event.key == 'a':
        new_theta[1] += 0.05
    elif event.key == 'd':
        new_theta[1] -= 0.05

    # Edge case for keeping the arm above the horizontal plane
    link1_y = lLengths[0] * np.sin(new_theta[0])
    link2_y = link1_y + lLengths[1] * np.sin(new_theta[0] + new_theta[1])
    if link1_y >= 0 and link2_y >= 0:
        theta[:] = new_theta
        state = update_SS(state, stateMatrix, inputMatrix, dtheta)

    update_robot()

'''
    @update_robot: updates the position of the links in the simulation

    @params: none
    @returns: none
'''
def update_robot():
    global theta, theta_history, state
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

    # Update info window
    joint1_var.set(f"Joint 1: {np.degrees(theta[0]):.2f}°")
    joint2_var.set(f"Joint 2: {np.degrees(theta[1]):.2f}°")
    state_var.set(f"State: {state}")
    state_matrix_var.set(f"stateMatrix: {np.array2string(stateMatrix, precision=2, separator=',')}")
    input_matrix_var.set(f"inputMatrix: {np.array2string(inputMatrix, precision=2, separator=',')}")
    output_matrix_var.set(f"outputMatrix: {np.array2string(outputMatrix, precision=2, separator=',')}")
    feedthrough_matrix_var.set(f"feedthroughMatrix: {np.array2string(feedthroughMatrix, precision=2, separator=',')}")


'''
    @create_info(): creates the information window for the robot simulation
'''
def create_info():
    info_window = tk.Toplevel(root)
    info_window.title("2DoF Arm Information")

    # Create vars
    global joint1_var, joint2_var, state_var, state_matrix_var, input_matrix_var, output_matrix_var, feedthrough_matrix_var
    joint1_var = tk.StringVar()
    joint2_var = tk.StringVar()
    state_var = tk.StringVar()
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

    # Create labels for joint angles and state
    joint1_label = tk.Label(info_window, textvariable=joint1_var)
    joint2_label = tk.Label(info_window, textvariable=joint2_var)
    state_label = tk.Label(info_window, textvariable=state_var)
    joint1_label.grid(row=1, column=0, padx=10, pady=5)
    joint2_label.grid(row=1, column=1, padx=10, pady=5)
    state_label.grid(row=1, column=2, padx=10, pady=5)


'''
    @make_tkinter: Creates the tkinter window for modeling 2dof robot and information
    
    @params: none
    @return: none
'''
def make_tkinter():
    global root, frame, canvas, fig, ax
    root = tk.Tk()
    root.title("2Dof Arm")

    # Main frame for canvas
    frame = tk.Frame(root)
    frame.grid(row=0, column=0, columnspan=4, sticky="nsew")
    fig, ax = plt.subplots()

    # Create canvas
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    fig.canvas.mpl_connect('key_press_event', key_press)

    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)
    root.grid_columnconfigure(2, weight=1)
    root.grid_columnconfigure(3, weight=1)
    create_info()

theta_history = []

def main():
    make_tkinter()
    update_robot()
    root.mainloop()

if __name__ == "__main__":
    main()
