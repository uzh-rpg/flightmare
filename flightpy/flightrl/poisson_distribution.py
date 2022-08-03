import argparse
from random import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

from trajectory_generator import TrajectoryGenerator


class PoissonDistribution():

    def __init__(self, radius, type=1,  maximum_x=200, maximum_y=200, max_try=30, trajectories=None):

        ''' Current types: 
                          {{1, "beech_tree_04",}, 
                          {2, "prefab_beech_tree_06",}, 
                          {3, "prefab_beech_tree_07",}, 
                          {4, "prefab_beech_tree_09",},
                          {5, "prefab_beech_forest_stones_01_4",},
                          {6, "prefab_beech_forest_stones_01_5",},
                          {7, "prefab_beech_forest_stones_01_6",},
                          {8, "prefab_beech_forest_stones_01_7",},
                          {9, "rpg_box02",},
                          {10, "rpg_wall01",},
                          {11, "rpg_wall02",}};
        '''
        self.radius = radius
        self.type = type
        self.cell_size = self.radius / np.sqrt(2)
        self.max_try = max_try
        self.maximum_x = maximum_x
        self.maximum_y = maximum_y  # assumiing region of interest [0, maximum_x] x [0, maximum_y]

        # number of cells in both dimension
        self.nx, self.ny= int (self.maximum_x / self.cell_size) + 1, int (self.maximum_y / self.cell_size) + 1
        self.coords_list = [(ix, iy) for ix in range(self.nx) for iy in range(self.ny)]
        # None of the cell is occupied except
        self.cells = {coords: None for coords in self.coords_list}


        # initial active list point from which can be used as reference point to extend searching
        self.active = [0]
        initial_point = (np.random.uniform(0, self.maximum_x), np.random.uniform(0, self.maximum_y))
        self.samples = [initial_point]
        self.cells[self.get_cell_coord(initial_point)] = 0

        self.maximum_count = 500 # maximum number of trees to be set

        self.trajectories = trajectories
        self.collision_threshold = 1.0
        self.flightmare_tree_dim = 9

    def load_trajectories(self, trajectories):
        self.trajectories = trajectories
   
    def get_cell_coord(self, point):
        # return the coordinate in cell of a point (double type)
        return int(point[0] // self.cell_size), int(point[1] // self.cell_size)

    def create_list(self):
        num_samples = 0
        while self.active:
            # randomly choose a reference point from the active list
            ref_index =  np.random.choice(self.active)
            ref_point = self.samples[ref_index]
            point = self.generate_point(ref_point)
            if point:
                self.samples.append(point)
                self.active.append(len(self.samples) - 1)
                self.cells[self.get_cell_coord(point)] = len(self.samples) - 1
                num_samples += 1
                if num_samples > self.maximum_count:
                    break
            else:
                self.active.remove(ref_index)
        print("Generating A list of Length:", num_samples)


    def generate_point(self, ref_point):
        ''' generate a candidate point relative to the reference point'''
        try_counter = 0
        while try_counter < self.max_try:
            try_counter += 1
            rho = np.sqrt(np.random.uniform(self.radius**2, 4 * self.radius**2))
            theta = np.random.uniform(0, 2*np.pi)
            candidate = ref_point[0] + rho * np.cos(theta), ref_point[1]  + rho * np.sin(theta)
            if self.is_valid(candidate) and self.is_collision_free(candidate):
                return candidate
        return False

    def is_valid(self, candidate):
        '''
        check if a candidate is valid sample, to be a valid sample, it has to 
        1) lie in the valida region;
        2) it's distance to the closest existing sample should be at least radius.
        '''
        if candidate[0] >= 0 and candidate[0] < self.maximum_x and candidate[1] >= 0 and candidate[1] < self.maximum_y:
            cell_coord = self.get_cell_coord(candidate)
            for idx in self.get_neighboring_points(cell_coord):
                point_pos = self.samples[idx]
                dst2 = (point_pos[0] - candidate[0]) ** 2 + (point_pos[1] - candidate[1]) ** 2
                if dst2 < self.radius ** 2:
                    return False
            return True
        else:
            return False
    
    def is_collision_free(self, candidate):
        if not self.trajectories:
            return True
        else:
            for trajectory in self.trajectories:
                for i in range(trajectory.shape[0]):
                    point_pos = trajectory[i]
                    dst2 = (point_pos[0] - candidate[0]) ** 2 + (point_pos[1] - candidate[1]) ** 2
                    if dst2 < self.collision_threshold ** 2:
                        return False
            return True

    def get_neighboring_points(self, coord):
        dxdy = [(-1,-2),(0,-2),(1,-2),(-2,-1),(-1,-1),(0,-1),(1,-1),(2,-1),
            (-2,0),(-1,0),(1,0),(2,0),(-2,1),(-1,1),(0,1),(1,1),(2,1),
            (-1,2),(0,2),(1,2),(0,0)]
        neighbors = []
        for dx, dy in dxdy:
            neighbor_coords = coord[0] + dx, coord[1] + dy
            if not (0 <= neighbor_coords[0] < self.nx and 0 <= neighbor_coords[1] < self.ny):
                continue
            neighbor_cell = self.cells[neighbor_coords]
            if neighbor_cell is not None:
                # if a neighbor is occupied, store the index of the sample it contains
                neighbors.append(neighbor_cell)
        return neighbors
    
    def generate_trees(self):
        trees = np.zeros((len(self.samples), self.flightmare_tree_dim))
        for i in range(len(self.samples)):
            x, y = self.samples[i]
            z = np.random.uniform(-0.1, 0.0)
            yaw = np.random.uniform(0, 2 * np.pi)
            pitch = np.random.uniform(-0.1, 0.1)
            roll = np.random.uniform(-0.1, 0.1)
            rot = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)
            quat = rot.as_quat()
            # scale = np.random.uniform(15.0, 20.2)
            scale = np.random.uniform(0.8, 2.0) if self.type <= 4 else np.random.uniform(20.0, 30.2)
            object_type = self.type
            trees[i,:] = np.array((x, y, z, quat[3], quat[0], quat[1], quat[2], scale, object_type))
        return trees
    
    def plot_samples(self):
        plt.scatter(*zip(*self.samples), color='r', alpha=0.6)
        if self.trajectories:
            for trajectory in self.trajectories:
                plt.plot(trajectory[:, 0], trajectory[:, 1])
        # plt.xlim(0, self.maximum_x)
        # plt.ylim(0, self.maximum_y)
        plt.axis('off')
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser("""Generate Poisson Distribution Points""")
    args = parser.parse_args()

    trajectory_generator = TrajectoryGenerator(80, 80, 10, 1/2, 100)
    trajectories = trajectory_generator.generate_trajectory()
    poisson_distribution = PoissonDistribution(5, 100, 100, 30, trajectories)
    poisson_distribution.create_list()
    poisson_distribution.plot_samples()
    # trees = poisson_distribution.generate_trees()
    # print(trees)

