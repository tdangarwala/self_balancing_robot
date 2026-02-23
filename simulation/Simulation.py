import control as ct
import numpy as np
import matplotlib.pyplot as plt

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

    def _run_simulation(self, runtime, dt, x0, disturbance_fn=None):
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