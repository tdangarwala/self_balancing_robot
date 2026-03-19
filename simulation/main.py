import numpy as np

from RobotParams import RobotParams
from Simulation import Simulation, run_ga

if __name__ == "__main__":


    xmax = 0.1  # max position (m)
    xdot_max = 0.1
    theta_max = np.deg2rad(10)  # max angle (radians)
    thetadot_max = 2

    input_max = 2

    #Q = np.diag([1/xmax**2, 1/xdot_max**2, 1/theta_max**2, 1/thetadot_max**2])
    #R = np.array([[1/input_max**2]])

    Q = np.diag([1, 1, 10, 1])
    R = np.array([[1.0]])
    params = RobotParams()

    #motor time constant = J/B

    best_Q, best_R, best_score = run_ga(params, tau=0.1, generations=50, pop_size=40)

    sim = Simulation(params, best_Q, best_R, tau=0.1)

    #sim.run_nonlinear_balancing(runtime=10, dt=0.001)

    sim.run_regular_balancing(runtime=10, dt=0.001)
    # sim.run_poke_force_simulation(runtime=10, dt=0.001)
    # sim.run_object_balancing_simulation(runtime=10, dt=0.001)