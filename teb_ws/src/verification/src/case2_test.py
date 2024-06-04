from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data = [
   
    
    {'V': np.array([[18.37316, 0.281, 0., 0.],
                     [-1.83922, 0., 0.306, 0.],
                     [2.46924, 0., 0., 0.001]])},
    {'V': np.array([[18.37316, 0.281, 0., 0.],
                     [-1.83922, 0., 0.306, 0.],
                     [2.46924, 0., 0., 0.001]])},
    {'V': np.array([[18.37316, 0.281, 0., 0.],
                     [-1.83922, 0., 0.306, 0.],
                     [2.46924, 0., 0., 0.001]])},
    {'V': np.array([[18.37316, 0.281, 0., 0.],
                     [-1.83922, 0., 0.306, 0.],
                     [2.46924, 0., 0., 0.001]])},
   
    
    
    
    
    
    {'V': np.array([[16.06583, 0.281, 0., 0.],
                     [-1.85549, 0., 0.306, 0.],
                     [-0.11448, 0., 0., 0.001]])},
    
    {'V': np.array([[16.41723, 0.281, 0., 0.],
                     [-1.84815, 0., 0.306, 0.],
                     [-0.18216, 0., 0., 0.001]])},
    
    {'V': np.array([[ 16.76862, 0.281, 0., 0.],
                     [-1.84081, 0., 0.306, 0.],
                     [-0.24983, 0., 0., 0.001]])},
    
    
    {'V': np.array([[17.12002, 0.281, 0., 0.],
                     [-1.83348, 0., 0.306, 0.],
                     [-0.31751, 0., 0., 0.001]])}
   
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
    

    dir_mat = np.array([[1, 0.27], [0.27,1]])
    
   
    
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