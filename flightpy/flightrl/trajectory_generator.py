import argparse
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class TrajectoryGenerator():

    def __init__(self, x_max = 50, y_max = 50, r=2, delta=1, max_num_trajectory=10):
        self.x_max = x_max
        self.y_max = y_max
        self.r = r
        self.delta_theta = np.pi / 12
        self.max_try = 30
        # interpolate by there waypoints, time step between waypoints is delta
        self.t = [0, delta, 2 * delta]
        self.num_trajectory = max_num_trajectory

        # save coeffs to generate trajectory as a function of time
        self.coeffs = []

        # random velocity
        self.vz = 0.1
        self.wx = 0.3

    def generate_trajectory_coeff(self):
        '''
            each dimension we approximate the trajectory with a quadratic function of time
            x = ax * t ** 2 + bx * t + cx
            y = ay * t ** 2 + by * t + cy
        '''
        waypoints = self.generate_waypoint_in_xy_plane()
        if waypoints:
            x0, y0 = waypoints[0]
            x1, y1 = waypoints[1]
            x2, y2 = waypoints[2]

            cx = x0
            cy = y0

            ax = (x1 - x0) / (self.t[1] * (self.t[1] - self.t[2])) - (x1 - x2) / (self.t[1] - self.t[2])**2
            ay = (y1 - y0) / (self.t[1] * (self.t[1] - self.t[2])) - (y1 - y2) / (self.t[1] - self.t[2])**2

            bx = (x1 - x0) / (self.t[1] - self.t[2]) - ax * (self.t[1] + self.t[2])
            by = (y1 - y0) / (self.t[1] - self.t[2]) - ay * (self.t[1] + self.t[2])

            coeff = []
            coeff.append([ax, bx, cx])
            coeff.append([ay, by, cy])

            # generate random twist command for the drone, not too realistic, but to increase coverage of the observed scenario
            coeff.append(np.random.uniform(1.0, 2.0)) # offset in z
            coeff.append(np.random.uniform(-self.vz, self.vz))
            coeff.append(np.random.uniform(-self.wx, self.wx))

            return coeff
        else:
            return False
    
    def get_state(self, idx, t):
        ''' get the state [x(t), y(t), z(t), ] at time t for trajectory idx'''
        assert idx < len(self.coeffs)
        coeff = self.coeffs[idx]
        x_coeff = coeff[0]
        y_coeff = coeff[1]

        state = np.zeros(25)
        vx = 2 * x_coeff[0] + x_coeff[1]
        vy = 2 * y_coeff[0] + y_coeff[1]
        vz = coeff[2]

        state[1] = x_coeff[0] * t ** 2 + x_coeff[1] * t + x_coeff[2]
        state[2] = y_coeff[0] * t ** 2 + y_coeff[1] * t + y_coeff[2]
        state[3] = coeff[2] + coeff[3] * t
        yaw = math.atan2(vy, vx)
        roll = coeff[4] * t
        pitch = math.atan2(vz, np.sqrt(vx ** 2 + vy ** 2))
        rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        quat = rot.as_quat()
        state[4] = quat[3]
        state[5] = quat[0]
        state[6] = quat[1]
        state[7] = quat[2]

        return state
            
    def generate_trajectory(self):
        ''' generate trajectories in order to check collision status.'''
        trajectories = []
        time_stamps = np.arange(self.t[0], self.t[2], self.t[2] / 50)
        for i in range(self.num_trajectory):
            coeff = self.generate_trajectory_coeff()
            x_coeff = coeff[0]
            y_coeff = coeff[1]
            x_array = x_coeff[0] * time_stamps ** 2 + x_coeff[1] * time_stamps + x_coeff[2]
            y_array = y_coeff[0] * time_stamps ** 2 + y_coeff[1] * time_stamps + y_coeff[2]
            trajectory = np.concatenate((x_array.reshape(-1, 1), y_array.reshape(-1,1)), axis=1)
            if self.is_valid_trajectory(trajectory):
                trajectories.append(trajectory)
                self.coeffs.append(coeff)
        return trajectories
        # plt.plot(x_array, y_array)
        # plt.show()
        

    def generate_waypoint_in_xy_plane(self):
        ''' generate three waypoints (x0, y0), (x1, y1), (x2,y2)'''
        x0, y0 = (np.random.uniform(0, self.x_max), np.random.uniform(0, self.y_max))
        waypoints = [(x0, y0)]
        try_x1 = 0
        while try_x1 < self.max_try:
            try_x1 += 1
            rho = np.random.uniform(self.r * 0.8, self.r * 1.2)
            theta = np.random.uniform(0, 2 * np.pi)
            candidate = x0 + rho * np.cos(theta), y0 + rho * np.sin(theta)
            if self.is_valid(candidate):
                x1, y1 = candidate
                waypoints.append(candidate)
                # select x2
                x2_try = 0
                while x2_try < self.max_try:
                    x2_try += 1
                    rho = np.random.uniform(self.r * 0.8, self.r * 1.2)
                    theta2 = np.random.uniform(theta - self.delta_theta, theta + self.delta_theta)
                    candidate2 = x1 + rho * np.cos(theta2), y1 + rho * np.sin(theta2)
                    if self.is_valid(candidate2):
                        x2, y2 = candidate2
                        waypoints.append(candidate2)
                        return waypoints
        return False
    
    def is_valid(self, candidate):
        if candidate[0] >= 0 and candidate[0] < self.x_max and candidate[1] >= 0 and candidate[1] < self.y_max:
            return True
        else:
            return False
    
    def is_valid_trajectory(self, trajectory):
        for i in range(trajectory.shape[0]):
            candidate = trajectory[i]
            if not self.is_valid(candidate):
                return False
        return True
    
    def get_num_trajectories(self):
        return len(self.coeffs)
                

if __name__ == "__main__":
    parser = argparse.ArgumentParser("""Generate a trajectory within an area""")
    args = parser.parse_args()

    trajectory_generator = TrajectoryGenerator(50, 50, 7, 1)
    trajectories = trajectory_generator.generate_trajectory()
    print(trajectories[0])
