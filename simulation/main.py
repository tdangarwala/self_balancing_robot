import numpy as np

from RobotParams import RobotParams
from Simulation import Simulation

if __name__ == "__main__":


    xmax = 0.1  # max position (m)
    xdot_max = 0.1
    theta_max = np.deg2rad(10)  # max angle (radians)
    thetadot_max = 2

    input_max = 2

    #Q = np.diag([1/xmax**2, 1/xdot_max**2, 1/theta_max**2, 1/thetadot_max**2])
    #R = np.array([[1/input_max**2]])

    Q = np.diag([10, 1, 100, 1])
    R = np.array([[0.01]])

    #motor time constant = J/B

    sim = Simulation(RobotParams(), Q, R, tau=0.02)

    sim.run_regular_balancing(runtime=10, dt=0.001)
    sim.run_poke_force_simulation(runtime=10, dt=0.001)
    sim.run_object_balancing_simulation(runtime=10, dt=0.001)