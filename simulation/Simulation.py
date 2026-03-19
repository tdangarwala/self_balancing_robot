from operator import lt
from time import time
import control as ct
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from torch import threshold

from Analysis import Analysis

class Simulation:
    def __init__(self, RobotParams, Q, R, tau):
        self.RobotParams = RobotParams
        self.Q = Q
        self.R = R
        self.u_max = 1.765/self.RobotParams.rw #Tmax / rw
        self.tau = tau  # motor time constant  

        self.last_sensed_theta = 0.0  # For sensor noise filtering      

    def _set_state_space_matrices(self):
        Meff = self.RobotParams.mb + self.RobotParams.mw + self.RobotParams.iw/self.RobotParams.rw**2
        Ieff = self.RobotParams.mb*self.RobotParams.l**2 + self.RobotParams.ib

        M = np.array([[Meff, -self.RobotParams.mb*self.RobotParams.l],[-self.RobotParams.mb*self.RobotParams.l, Ieff]])

        detM = (Meff * Ieff) - (self.RobotParams.mb * self.RobotParams.l)**2

        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, -(self.RobotParams.mb**2 * self.RobotParams.g * self.RobotParams.l**2) / detM, 0], # Gravity effect on linear motion
            [0, 0, 0, 1],
            [0, 0, (Meff * self.RobotParams.mb * self.RobotParams.g * self.RobotParams.l) / detM, 0]  # Gravity effect on angular motion (unstable)
        ])

        # B matrix: Note the opposite signs in rows 2 and 4!
        self.B = np.array([
            [0],
            [Ieff / detM],
            [0],
            [-self.RobotParams.mb * self.RobotParams.l / detM] # Pushing wheels forward tilts body back
        ])

        #E = np.array([[0],[mb*l/np.linalg.det(M)],[0],[Meff/np.linalg.det(M)]])
        self.E = np.array([
            [0],
            [Ieff / detM],
            [0],
            [-self.RobotParams.mb * self.RobotParams.l / detM]
        ])

    def _is_controllable_observable(self):
        #check controllability
        ctrb = ct.ctrb(self.A,self.B)
        rank_ctrb = np.linalg.matrix_rank(ctrb)

        #check observability
        C = np.eye(self.A.shape[0]) 
        obsv = ct.obsv(self.A,C)
        rank_obsv = np.linalg.matrix_rank(obsv)
        

        if rank_ctrb == self.A.shape[0] and rank_obsv == self.A.shape[0]:
            return True
        else:
            return False
    
    def _create_LQR_controller(self):
        # Create LQR controller
        self.K, _, _ = ct.lqr(self.A, self.B, self.Q, self.R)
        print("LQR Gain K:", self.K)

    def _add_sensor_noise(self, x, use_noise=True):
        if not use_noise:
            return x
        
        lpf_alpha = 0.1      # Low-pass filter alpha for sensor noise
        noise = np.random.normal(0, np.deg2rad(0.1))
        theta_noisy = x[2,0] + noise
        self.last_sensed_theta = (lpf_alpha * theta_noisy) + ((1 - lpf_alpha) * self.last_sensed_theta)

        x_sensed = x.copy()
        x_sensed[2,0] = self.last_sensed_theta

        return x_sensed

    def _setup_controller(self):
        self._set_state_space_matrices()
        if not self._is_controllable_observable():
            raise RuntimeError("System is not controllable and/or observable.")
        
        self._create_LQR_controller()
        self.last_sensed_theta = 0.0  # Reset sensor noise filter state

    def _run_simulation(self, runtime, dt, x0, disturbance_fn=None, plot = False):
        analysis = Analysis()
        x = x0.copy()
        u_actual = np.array([[0.0]])

        time = []
        ucmd = []
        uactual = []

        for t in np.arange(0, runtime, dt):
            d = disturbance_fn(t) if disturbance_fn else np.zeros((self.E.shape[1],1))
            x_sensed = self._add_sensor_noise(x)

            u_cmd = -self.K @ x_sensed
            u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)

            u_actual += (u_cmd - u_actual) * dt/self.tau

            ucmd.append(u_cmd[0,0])
            uactual.append(u_actual[0,0])
            time.append(t)

            x_dot = self.A @ x + self.B @ u_actual + self.E @ d
            analysis.add([x[0,0], x[1,0], x[2,0], x[3,0], t])

            x += x_dot * dt
        
        if plot:

            plt.plot(time, np.array(ucmd), label='Commanded Input')
            plt.plot(time, np.array(uactual), label='Actual Input', linestyle='--')
            plt.title('Commanded vs Actual Input Torque')
            plt.xlabel('Time (s)')
            plt.ylabel('Input Torque (Nm)')
            plt.legend()

        return analysis

    def _run_nonlinear_simulation(self, runtime, dt, x0):
        analysis = Analysis()
        x = x0.copy()
        u_actual = np.array([[0.0]])

        stall_torque = 1.76
        no_load_speed = 26.3

        p = self.RobotParams
        Meff = p.mb + p.mw + p.iw/p.rw**2
        Ieff = p.mb*p.l**2 + p.ib

        for t in np.arange(0,runtime, dt):
            x_sensed = self._add_sensor_noise(x) #where am i via sensors

            v = x[1,0]
            w_wheel = v / p.rw

            max_phys_torque = stall_torque * (1 - abs(w_wheel) / no_load_speed)
            max_phys_torque = max(0, max_phys_torque)

            u_cmd = -self.K @ x_sensed #how do i react 
            u_cmd = np.clip(u_cmd, -self.u_max, self.u_max) #adjust reaction based on motor capabilities
            u_final = np.clip(u_cmd, -max_phys_torque, max_phys_torque)

            u_actual += (u_final - u_actual) * dt/self.tau #adjust reaction based on motor lag

            #using modeled state to calculate nonlinear dynamics 
            theta = x[2,0]
            theta_dot = x[3,0]
            f_motor = u_actual[0,0]

            # Nonlinear Mass Matrix M(q)
            M = np.array([
                [Meff, -p.mb * p.l * np.cos(theta)],
                [-p.mb * p.l * np.cos(theta), Ieff]
            ]) #negative for diagonals because moving positive in x direction tilts body back and vice versa

            G = np.array([
                [0],
                [-p.mb * p.g * p.l * np.sin(theta)]])
            
            C = np.array([
                [-p.mb*p.l*(theta_dot**2)*np.sin(theta)],
                [0]])
            
            B = np.array([
                [-f_motor],
                [0]
            ])

            # Bu + G - C -> +G because matrix has negative sign.
            rhs = B + G - C

            # Solve for accelerations [x_ddot, theta_ddot]
            # This replaces "x_dot = A@x + B@u"
            accels = np.linalg.solve(M, rhs)
        
            # 3. Integrate
            x_dot = np.array([[x[1,0]], [accels[0,0]], [x[3,0]], [accels[1,0]]])
            analysis.add([x[0,0], x[1,0], x[2,0], x[3,0], t])
            x += x_dot * dt


        return analysis

    def run_nonlinear_balancing(self, runtime, dt):
        self._setup_controller()
        x0=np.array([[0.0],[0.0],[np.deg2rad(10)],[0.2]])
        data = self._run_nonlinear_simulation(runtime,dt,x0)
        data.plot_results("Nonlinear Balancing Simulation Results")
        
    def run_regular_balancing(self, runtime, dt):
        self._setup_controller()
        x0=np.array([[0.0],[0.0],[np.deg2rad(5)],[0.2]])
        data = self._run_simulation(runtime,dt,x0)
        data.plot_results("Regular Balancing Simulation Results")

    def run_poke_force_simulation(self, runtime, dt):
        self._setup_controller()
        def poke_disturbance(t):
            if 5 <= t < 5.1:
                return np.array([[10.0]])
            else:
                return np.zeros((self.E.shape[1],1))
        
        x0=np.array([[0.0],[0.0],[0.0],[0.0]])
        data = self._run_simulation(runtime,dt,x0,disturbance_fn=poke_disturbance)
        data.plot_results("Poke Force Simulation Results")

    def run_object_balancing_simulation(self, runtime, dt, extra_mass=[0.5, 8.0], plot_combined=True):
        self._setup_controller()
        original_mb = self.RobotParams.mb

        all_run_data = []

        analysis = Analysis()

        for mass in np.arange(extra_mass[0], extra_mass[1]+0.1, 1.0):
            self.RobotParams.mb = original_mb + mass
            self.RobotParams.ib = (1/3) * self.RobotParams.mb * self.RobotParams.l**2                
            self._set_state_space_matrices()
            
            x0 = np.array([[0.0],[0.0],[np.deg2rad(10)],[0.2]])

            data = self._run_simulation(runtime,dt,x0)

            all_run_data.append((f"+{mass:.1f}kg", data.get_data()))

            self.RobotParams.mb = original_mb # Reset for next loop

        self.RobotParams.mb = original_mb

        if plot_combined:
            analysis.plot_multiple_runs(all_run_data, "Object Balancing: All Mass Variations")

def settling_time(time, signal, threshold_deg=2.0):
    """
    Return the last timestamp at which |signal| exceeded threshold_deg.
    This is how long the robot took to truly settle.
    Returns 0.0 if the signal never left the threshold band (already settled).
    """
    threshold = np.deg2rad(threshold_deg)
    exceeded = np.where(np.abs(signal) > threshold)[0]
    if len(exceeded) == 0:
        return 0.0
    return time[exceeded[-1]]


def evaluate(genome, robot_params, tau):
    """
    Score one genome (a set of Q and R values).

    The genome is 5 floats in log10 space:
        [log(q1), log(q2), log(q3), log(q4), log(r1)]

    We decode to real values, build Q and R, run two simulation
    scenarios, and return a scalar score. Higher is better.
    A score of -1e9 means the robot fell over — discard.

    Scenarios:
        1. Poke: robot starts upright, hit with 10N at t=5s
        2. Tilt: robot starts tilted 10 degrees with some angular velocity
    """
    # Decode log-space genome back to real Q/R values
    # e.g. genome value 2.0  → 10^2.0 = 100
    #      genome value -1.0 → 10^-1.0 = 0.1
    q1, q2, q3, q4, r1 = [10 ** v for v in genome]

    Q = np.diag([q1, q2, q3, q4])
    R = np.array([[r1]])

    try:
        sim = Simulation(robot_params, Q, R, tau)
        sim._setup_controller()

        # --- Scenario 1: poke disturbance ---
        def poke(t):
            return np.array([[10.0]]) if 5 <= t < 5.1 else np.zeros((1, 1))

        poke_data = sim._run_simulation(
            runtime=10, dt=0.01,
            x0=np.array([[0.], [0.], [0.], [0.]]),
            disturbance_fn=poke,
            plot=False           # no matplotlib windows during GA
        ).get_data()

        # --- Scenario 2: initial tilt (simulates object placed on top) ---
        tilt_data = sim._run_simulation(
            runtime=10, dt=0.01,
            x0=np.array([[0.], [0.], [np.deg2rad(10)], [0.2]]),
            plot=False
        ).get_data()

    except Exception:
        # LQR failed to solve, or system went unstable — worst score
        return -1e9

    theta_poke = poke_data['theta']
    theta_tilt = tilt_data['theta']
    time_arr   = poke_data['time']

    # --- Hard failure check: fell over (>45 degrees) ---
    if np.max(np.abs(theta_poke)) > np.deg2rad(45):
        return -1e9
    if np.max(np.abs(theta_tilt)) > np.deg2rad(45):
        return -1e9

    # --- Metrics ---
    # 1. Settling time: how long until theta stays within 2 degrees
    poke_settle = settling_time(time_arr, theta_poke)
    tilt_settle = settling_time(tilt_data['time'], theta_tilt)

    # 2. Overshoot: peak angle deviation
    poke_overshoot = np.max(np.abs(theta_poke))
    tilt_overshoot = np.max(np.abs(theta_tilt))

    # 3. Control effort: penalise rapid velocity changes (proxy for motor thrashing)
    poke_effort = np.sum(np.abs(np.diff(poke_data['xdot'])))
    tilt_effort = np.sum(np.abs(np.diff(tilt_data['xdot'])))

    # --- Combined score (negative because we maximise, but want small values) ---
    # Weights: tweak these to change what "good" means to you.
    #   3.0  → settling speed is important
    #   5.0  → overshoot is most important (don't swing wildly)
    #   0.1  → effort matters a little (don't thrash motors)
    score = -(
        3.0 * (poke_settle   + tilt_settle)    +
        5.0 * (poke_overshoot + tilt_overshoot) +
        0.1 * (poke_effort   + tilt_effort)
    )

    return score


def run_ga(robot_params, tau, generations=50, pop_size=40, seed=None):
    """
    Run the genetic algorithm to find optimal Q and R matrices.

    Args:
        robot_params:  Your RobotParams object.
        tau:           Motor time constant.
        generations:   How many generations to evolve (50 is a good start).
        pop_size:      How many individuals per generation (40 is a good start).
        seed:          Optional random seed for reproducibility.

    Returns:
        best_Q, best_R, best_score

    Example usage:
        best_Q, best_R, score = run_ga(robot_params, tau=0.1)
        sim = Simulation(robot_params, best_Q, best_R, tau=0.1)
        sim.run_poke_force_simulation(runtime=10, dt=0.01)
    """
    if seed is not None:
        np.random.seed(seed)

    genome_size = 5  # [log(q1), log(q2), log(q3), log(q4), log(r1)]

    # Search space: each value is in log10 space.
    # [-3, 3] means actual Q/R values range from 0.001 to 1000.
    # State vector: [x, x_dot, theta, theta_dot]
    # Sensible starting intuition:
    #   q1 (position)        — low weight, we don't mind drifting a bit
    #   q2 (velocity)        — medium weight
    #   q3 (angle)           — HIGH weight, this is the critical state
    #   q4 (angular vel)     — medium-high weight
    #   r1 (control effort)  — low weight, motors are strong enough
    bounds = np.array([
        [-3.0, 3.0],   # log(q1): position
        [-3.0, 3.0],   # log(q2): velocity
        [-1.0, 4.0],   # log(q3): angle — biased toward higher values
        [-2.0, 3.0],   # log(q4): angular velocity
        [-3.0, 1.0],   # log(r1): control cost — biased toward lower values
    ])

    # --- Initial population: random within bounds ---
    population = np.random.uniform(
        bounds[:, 0], bounds[:, 1],
        size=(pop_size, genome_size)
    )

    best_genome = None
    best_score  = -np.inf

    print(f"Starting GA: {generations} generations × {pop_size} individuals "
        f"= {generations * pop_size} total evaluations")
    print("-" * 55)

    for gen in range(generations):
        # --- Evaluate every individual in this generation ---
        scores = [evaluate(g, robot_params, tau) for g in population]

        # --- Track the best genome seen so far ---
        gen_best_idx = int(np.argmax(scores))
        if scores[gen_best_idx] > best_score:
            best_score  = scores[gen_best_idx]
            best_genome = population[gen_best_idx].copy()

        # --- Print progress every 5 generations ---
        if gen % 5 == 0 or gen == generations - 1:
            q_vals = [round(10 ** v, 4) for v in best_genome[:4]]
            r_val  = round(10 ** best_genome[4], 4)
            print(f"Gen {gen:3d} | best score: {best_score:8.4f} | "
                f"Q=diag({q_vals}) R=[[{r_val}]]")

        # --- Selection: keep top 50% as parents ---
        sorted_idx = np.argsort(scores)[::-1]
        parents    = population[sorted_idx[:pop_size // 2]]

        # --- Breed next generation via crossover + mutation ---
        children = []
        while len(children) < pop_size:
            # Pick two distinct parents at random
            idx = np.random.choice(len(parents), size=2, replace=False)
            a, b = parents[idx[0]], parents[idx[1]]

            # Crossover: each gene comes from either parent with 50/50 chance
            mask  = np.random.rand(genome_size) > 0.5
            child = np.where(mask, a, b)

            # Mutation: add small gaussian noise to each gene
            # std=0.15 in log space means roughly ±40% change in real value
            child += np.random.normal(0, 0.15, genome_size)

            # Clamp back into the allowed search space
            child = np.clip(child, bounds[:, 0], bounds[:, 1])
            children.append(child)

        population = np.array(children)

    # --- Decode and return the winner ---
    q1, q2, q3, q4, r1 = [10 ** v for v in best_genome]
    best_Q = np.diag([q1, q2, q3, q4])
    best_R = np.array([[r1]])

    print("\n" + "=" * 55)
    print("GA complete.")
    print(f"Best Q: diag({[round(v, 4) for v in [q1, q2, q3, q4]]})")
    print(f"Best R: [[{r1:.4f}]]")
    print(f"Best score: {best_score:.4f}")
    print("=" * 55)

    return best_Q, best_R, best_score



#intuitions
# so the way im understanding these matrices is 
# A describes the physics of the system itself. What 
# forces are acting on the robot at all times and how 
# they relate to the states. So it's like how gravity '
# 'impacts velocity and angular velocity. The B matrix is how '
# 'the robot responds to any input like motor torque, but more '
# 'specifically it's how each state is impacted by an input 
# force from the motor. So the velocity is in the positive 
# direction and the angular velocity is in the other direction. 
# Same for the E matrix which is how each state reacts to a 
# disturbance force (d). C is what we can measure from the system. 
# D I still dont underestand well. 
# determinant is basically tells us how much ....