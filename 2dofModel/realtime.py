import numpy as np
import time
import threading
from luenberger_2DOF import *

# GLOBAL VAR
stop_simulation = False

def real_time_loop(state, state_estimate, u, dt, stateMatrix, inputMatrix, outputMatrix, observer_gain,
                   update_ss_func, update_gui_func):
    global stop_simulation
    while not stop_simulation:
        y = outputMatrix @ state
        state_estimate = update_observer(state_estimate, stateMatrix, inputMatrix, outputMatrix, observer_gain, u, y, dt)
        state = update_ss_func(state, stateMatrix, inputMatrix, u, dt)
        #print(f"State: {state}, Estimated State: {state_estimate}")
        time.sleep(dt)
        update_gui_func(state, state_estimate)
