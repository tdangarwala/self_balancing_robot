import matplotlib.pyplot as plt
import numpy as np

class Analysis:
    def __init__(self):
        self.x = []
        self.xdot = []
        self.theta = []
        self.thetadot = []
        self.time = []
    
    def add(self, state):
        self.x.append(state[0])
        self.xdot.append(state[1])
        self.theta.append(state[2])
        self.thetadot.append(state[3])
        self.time.append(state[4])

    def get_data(self):
        """Return simulation data as numpy arrays"""
        return {
            'time': np.array(self.time),
            'x': np.array(self.x),
            'xdot': np.array(self.xdot),
            'theta': np.array(self.theta),
            'thetadot': np.array(self.thetadot),
        }
    
    def calculate_trise(self):
        #Calculate rise time of velocity response to gauge whether controller is realistic 
        xdot_array = np.array(self.xdot)
        time_array = np.array(self.time)
        max = np.max(xdot_array)
        threshold_10 = 0.1 * max
        threshold_90 = 0.9 * max
        try:
            t_10_index = np.where(xdot_array >= threshold_10)[0][0]
            t_90_index = np.where(xdot_array >= threshold_90)[0][0]
            t_rise = time_array[t_90_index] - time_array[t_10_index]
            return t_rise
        except IndexError:
            return None  # Rise time could not be determined

    def plot_results(self, title):
        """Plot results for a single simulation run"""
        plt.figure(figsize=(12, 8))
        plt.suptitle(title, fontsize=16)

        plt.subplot(2, 2, 1)
        plt.plot(self.time, self.x)
        plt.title('Position (x) vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')

        plt.subplot(2, 2, 2)
        plt.plot(self.time, self.xdot)
        print(self.calculate_trise())
        plt.title(f'Velocity (xdot) vs Time Trise={self.calculate_trise():.2f}s')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')

        plt.subplot(2, 2, 3)
        plt.plot(self.time, self.theta)
        plt.title('Angle (theta) vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (rad)')

        plt.subplot(2, 2, 4)
        plt.plot(self.time, self.thetadot)
        plt.title('Angular Velocity (thetadot) vs Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')

        plt.tight_layout()
        plt.show()

    def plot_multiple_runs(self, run_data_list, title):
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(title, fontsize=16)

        for label, data in run_data_list:
            axes[0, 0].plot(data['time'], data['x'], label=label, alpha=0.7)
            axes[0, 1].plot(data['time'], data['xdot'], label=label, alpha=0.7)
            axes[1, 0].plot(data['time'], data['theta'], label=label, alpha=0.7)
            axes[1, 1].plot(data['time'], data['thetadot'], label=label, alpha=0.7)

        axes[0, 0].set_title('Position (x) vs Time')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Position (m)')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].set_title('Velocity (xdot) vs Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Velocity (m/s)')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].set_title('Angle (theta) vs Time')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Angle (rad)')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].set_title('Angular Velocity (thetadot) vs Time')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Angular Velocity (rad/s)')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()