from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data = [
    {'V': np.array([[11.46598, 0.281, 0., 0.],
                     [-7.26628, 0., 0.306, 0.],
                     [1.62037, 0., 0., 0.001]])},
    
    # {'V': np.array([[8.14437, 0.281, 0., 0.],
    #                  [0.03237, 0., 0.306, 0.],
    #                  [-0.30634, 0., 0., 0.001]])},
    
    
    
    {'V': np.array([[11.46594, 0.281, 0., 0.],
                     [-7.26553, 0., 0.306, 0.],
                     [1.62002, 0., 0., 0.001]])},
    
    # {'V': np.array([[ 8.1446, 0.281, 0., 0.],
    #                  [0.03229, 0., 0.306, 0.],
    #                  [-0.30493, 0., 0., 0.001]])},
    
    
    {'V': np.array([[11.4659, 0.281, 0., 0.],
                     [-7.26479, 0., 0.306, 0.],
                     [1.61966, 0., 0., 0.001]])}
    
    # {'V': np.array([[8.14483, 0.281, 0., 0.],
    #                  [ 0.03222, 0., 0.306, 0.],
    #                  [-0.30351, 0., 0., 0.001]])}
    ]

    points = []
    for item in data:
        V = item['V']
        x = V[0][0]
        y = V[1][0]
        points.append([x, y])

   
    points_array = np.array(points)

    print(points_array)
    
    p = []
    
    # angle = np.radians(45)  # Convert 45 degrees to radians
    # dir_mat = np.array([[1/np.cos(angle), np.sin(angle)],
    #                     [-np.sin(angle), 1/np.cos(angle)]])
    dir_mat = np.array([[1, 0], [0,1]])
    
   
    
    dir_vec = np.array([0, 0])
    
    for i in range(len(points_array)):
        c = (np.expand_dims(points_array[i], axis=0)).transpose()
        # print(c.shape)
        std = np.array([0.281,0.306])
        v = np.diag(np.array([0.281,0.306]))
        V = np.concatenate([c, v], axis =1)
        # print(V)
        C = []
        d = []
        mu = np.ones(2)
        sig = np.diag(np.ones(2))
        lb = np.array([-4.5,-4.5])
        ub = np.array([4.5,4.5])
        prob = ProbStar(V, C, d, mu, sig, lb, ub)
        p.append(prob)
    plot_probstar(p, dir_mat=dir_mat, dir_vec=dir_vec)
        
    
    
if __name__ == "__main__":
    plot()