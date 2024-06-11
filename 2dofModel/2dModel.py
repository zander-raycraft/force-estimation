import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from state_space_2DOF import *

'''
    @PARAMS:
        Link length (m) - l1, l2
        Link mass (kg) - m1, m2
        gravity acceleration (m/s^2) - g
        center of mass for links - r1, r2
        moment of Inertia for links - I1, I2
'''
l1 = 1.0
l2 = 1.0
m1 = 1.0
m2 = 1.0
g = 9.81
r1 = l1 / 2
r2 = l2 / 2
I1 = (1 / 3) * m1 * l1 ** 2
I2 = (1 / 3) * m2 * l2 ** 2

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

'''
    @key_press: converts keyboard input into movements for 2d arm

    @params: none
    @returns: none
'''
def key_press(event):
    global theta
    new_theta = theta.copy()
    if event.key == 'left':
        new_theta[0] += 0.05
    elif event.key == 'right':
        new_theta[0] -= 0.05
    elif event.key == 'a':
        new_theta[1] += 0.05
    elif event.key == 'd':
        new_theta[1] -= 0.05

    # edge case for keeping the arm above the horizontal plane
    link1_y = l1 * np.sin(new_theta[0])
    link2_y = link1_y + l2 * np.sin(new_theta[0] + new_theta[1])
    if link1_y >= 0 and link2_y >= 0:
        theta[:] = new_theta
    update_robot()

'''
    @update_robot: updates the position of the links in the simulation

    @params: none
    @returns: none
'''
def update_robot():
    global theta, theta_history

    # Theta storage
    theta_history.append(np.copy(theta))

    # Plot update
    ax.clear()
    x = [0, l1 * np.cos(theta[0]), l1 * np.cos(theta[0]) + l2 * np.cos(theta[0] + theta[1])]
    y = [0, l1 * np.sin(theta[0]), l1 * np.sin(theta[0]) + l2 * np.sin(theta[0] + theta[1])]
    ax.plot(x, y, '-o', linewidth=2)
    ax.plot([-2, 2], [0, 0], 'k--')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    canvas.draw()

    # Update joint angle labels
    joint1_var.set(f"Joint 1: {np.degrees(theta[0]):.2f}°")
    joint2_var.set(f"Joint 2: {np.degrees(theta[1]):.2f}°")

'''
    @make_tkinter: Creates the tkinter window for modeling 2dof robot and information
    
    @params: none
    @return: none
'''
def make_tkinter():
    # window creation
    global root, frame, canvas, fig, ax, joint1_var, joint2_var
    root = tk.Tk()
    root.title("2Dof Arm")

    # frame creation
    frame = tk.Frame(root)
    frame.grid(row=0, column=0, columnspan=2, sticky="nsew")
    fig, ax = plt.subplots()

    # canvas creation
    canvas = FigureCanvasTkAgg(fig, master=frame)
    canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
    fig.canvas.mpl_connect('key_press_event', key_press)

    # label creation
    joint1_var = tk.StringVar()
    joint2_var = tk.StringVar()
    joint1_label = tk.Label(root, textvariable=joint1_var)
    joint2_label = tk.Label(root, textvariable=joint2_var)
    joint1_label.grid(row=1, column=0, padx=10, pady=5)
    joint2_label.grid(row=1, column=1, padx=10, pady=5)
    root.grid_rowconfigure(0, weight=1)
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=1)

theta_history = []
def main():
    make_tkinter()
    update_robot()
    root.mainloop()


if __name__ == "__main__":
    main()

