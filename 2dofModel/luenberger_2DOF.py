import numpy as np
from scipy.linalg import solve_continuous_are


'''
    @calc_observerGain: uses place poles to calculate gain matrix for LO
    
    @params: A (np.array) - state matrix
    @params: C (np.array) - output matrix
    @params: Q (np.array) - noise covariance matrix
    @params: R (np.array) - measurement noise covariance matrix
    
    @return: gainMatrix (np.array): observer gain matrix
'''
def calculate_observer_gain(A, C, Q, R):

    # Using Riccati equation (ARE)
    P = solve_continuous_are(A.T, C.T, Q, R)
    gainMatrix = P @ C.T @ np.linalg.inv(R)
    return gainMatrix

'''
    @update_observer: updates the observer using Luenberger observer
    
    @params: A (np.array) - state matirx
    @params: B (np.array) - Input matrix
    @params: C (np.array) - output matrix
    @params: gain (np.array) - observer gain matrix
    @params: u (np.array) - control input vector
    @params: y (np.array) - Measured input vector
    @params: dt (float) - timestep (s)
    
    @returns: upStateEstimate (np.array) - updated state estimate
'''
def update_observer(state_estimate, A, B, C, gain, u, y, dt):
    y_estimate = C @ state_estimate
    x_dot_estimate = (A @ state_estimate) + (B @ u) + L @ (y - y_estimate)
    new_state_estimate = state_estimate + dt * x_dot_estimate
    return new_state_estimate
