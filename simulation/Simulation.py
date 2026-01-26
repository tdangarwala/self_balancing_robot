import control as ct
import numpy as np

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

        for t in np.arange(0, runtime, dt):
            d = disturbance_fn(t) if disturbance_fn else np.zeros((self.E.shape[1],1))
            x_sensed = self._add_sensor_noise(x)

            u_cmd = -self.K @ x_sensed
            u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)

            u_actual += (u_cmd - u_actual) * dt/self.tau

            x_dot = self.A @ x + self.B @ u_actual + self.E @ d
            analysis.add([x[0,0], x[1,0], x[2,0], x[3,0], t])

            x += x_dot * dt

        return analysis
        
    def run_regular_balancing(self, runtime, dt):
        self._setup_controller()
        x0=np.array([[0.0],[0.0],[np.deg2rad(10)],[0.2]])
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





    ######OLD INEFFICIENT CODE. KEEP FOR REFERENCE######

    # def run_regular_balancing(self, runtime, dt):
    #     #create A,B,E matrices based on robot params
    #     self._set_state_space_matrices()
    #     #check if system is controllable and observable
    #     check = self._is_controllable_observable()
    #     #only do LQR if system is controllable and observable
    #     if check:
    #         self._create_LQR_controller()
    #     else:
    #         print("System is not controllable and/or observable. Cannot create LQR controller.")
    #         return
        
    #     #init analysis object
    #     analysis = Analysis()
    #     self.last_sensed_theta = 0.0  # Reset sensor noise filter state

    #     #initial state of robot is stationary but tilting slightly
    #     x_regular = np.array([[0.0],[0.0],[np.deg2rad(10)],[0.2]])  # initial state: small angle deviation
    #     u_actual = np.array([[0.0]])  # initial actual input. no motor input at this time

    #     for t in np.arange(0, runtime, dt):
    #         d = np.zeros((self.E.shape[1],1))  # no disturbance for regular balancing
    #         x_sensed = self._add_sensor_noise(x_regular) # sensor noise added to system state as MPU6050 is known to be noisy
            
    #         u_cmd = -self.K @ x_sensed  # LQR control input with sensor noise incorporated
    #         u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)  # enforce input limits based on motor specs

    #         u_actual += (u_cmd - u_actual) * dt/self.tau  # first order lag for motor response

    #         x_dot = self.A @ x_regular + self.B @ u_actual + self.E @ d # simulate system dynamics, physics + actual inputs + disturbances(0)

    #         analysis.add([x_regular[0,0], x_regular[1,0], x_regular[2,0], x_regular[3,0], t]) #add to analysis object for plotting later

    #         x_regular += x_dot * dt #euler integration step to move simulation forward in time

    #     print("Final state after regular balancing simulation:", x_regular.flatten())
    #     analysis.plot_results("Regular Balancing Simulation Results") #plot results

    # def run_poke_force_simulation(self, runtime, dt):
    #     self._set_state_space_matrices()
    #     check = self._is_controllable_observable()
    #     if check:
    #         self._create_LQR_controller()
    #     else:
    #         print("System is not controllable and/or observable. Cannot create LQR controller.")
    #         return
        
    #     #init analysis object
    #     analysis = Analysis()

    #     self.last_sensed_theta = 0.0  # Reset sensor noise filter state

    #     #simulate poke force input
    #     x_poke = np.array([[0.0],[0.0],[0],[0]])  # initial state: upright
    #     u_actual = np.array([[0.0]])  # initial actual input

    #     for t in np.arange(0, runtime, dt):
    #         x_sensed = self._add_sensor_noise(x_poke)
    #         #simulate system dynamics here
    #         if 5 <= t < 5.1:
    #             d = np.array([[10.0]])  # apply a poke force disturbance for 0.1s
    #         else:
    #             d = np.zeros((self.E.shape[1],1))  # no disturbance otherwise
            
    #         u_cmd = -self.K @ x_sensed  # LQR control input
    #         u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)  # enforce input limits

    #         u_actual += (u_cmd - u_actual) * dt/self.tau  # first order lag for motor response

    #         x_dot = self.A @ x_poke + self.B @ u_actual + self.E @ d

    #         analysis.add([x_poke[0,0], x_poke[1,0], x_poke[2,0], x_poke[3,0], t])

    #         x_poke += x_dot * dt
        
    #     print("Final state after poke force simulation:", x_poke.flatten())
    #     analysis.plot_results("Poke Force Simulation Results")

    # def run_object_balancing_simulation(self, runtime, dt, extra_mass=[0.5, 8.0], plot_combined=True):
    #     self._set_state_space_matrices()
    #     check = self._is_controllable_observable()
    #     if check:
    #         self._create_LQR_controller() 
    #     else:
    #         print("System is not controllable and/or observable. Cannot create LQR controller.")
    #         return
    #     original_mb = self.RobotParams.mb

    #     # --- REALISM PARAMETERS ---
    #     mu = 0.7              # Friction coefficient (Rubber on Tile)
    #     self.last_sensed_theta = 0.0  # Reset sensor noise filter state
        
    #     all_run_data = []

    #     for mass in np.arange(extra_mass[0], extra_mass[1]+0.1, 1.0):
    #         self.RobotParams.mb = original_mb + mass
    #         self.RobotParams.ib = (1/3) * self.RobotParams.mb * self.RobotParams.l**2                
    #         self._set_state_space_matrices()

    #         # Friction limit for this specific payload
    #         total_mass = self.RobotParams.mb + self.RobotParams.mw
    #         f_max = mu * total_mass * self.RobotParams.g

    #         analysis = Analysis()
    #         x_object = np.array([[0.0],[0.0],[np.deg2rad(10)],[0.2]])
    #         u_actual = np.array([[0.0]])
            

    #         for t in np.arange(0, runtime, dt):
                
    #             # 2. ADD SENSOR NOISE
    #             x_sensed = self._add_sensor_noise(x_object)

    #             # 3. COMPUTE CONTROL
    #             u_cmd = -self.K @ x_sensed
    #             u_cmd = np.clip(u_cmd, -self.u_max, self.u_max)
                
    #             # Motor lag
    #             u_actual += (u_cmd - u_actual) * dt/self.tau

    #             # 4. ADD FRICTION (TRACTION) LIMIT
    #             # If required force > traction, wheels slip (force is capped)
    #             # applied_force = u_cmd / self.RobotParams.rw
    #             # if abs(applied_force) > f_max:
    #             #     u_phys = np.sign(u_cmd) * (f_max * self.RobotParams.rw)
    #             # else:
    #             #     u_phys = u_cmd

    #             # 5. PHYSICS STEP
    #             x_dot = self.A @ x_object + self.B @ u_actual
    #             x_object += x_dot * dt
                
    #             analysis.add([x_object[0,0], x_object[1,0], x_object[2,0], x_object[3,0], t])

    #         label = f"+{mass:.1f}kg"
    #         all_run_data.append((label, analysis.get_data()))
    #         self.RobotParams.mb = original_mb # Reset for next loop
        
    #     if plot_combined:
    #         Analysis.plot_multiple_runs(all_run_data, "Object Balancing: All Mass Variations")