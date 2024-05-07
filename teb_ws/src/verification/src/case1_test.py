from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data = [
    {'V': np.array([[9.16214, 0.281, 0., 0.],
                     [-1.22479, 0., 0.306, 0.],
                     [2.28944, 0., 0., 0.001]])},
    
    {'V': np.array([[8.15162, 0.281, 0., 0.],
                     [0.03028, 0., 0.306, 0.],
                     [-0.3251, 0., 0., 0.001]])},
    {'V': np.array([[8.15162, 0.281, 0., 0.],
                     [0.03028, 0., 0.306, 0.],
                     [-0.3251, 0., 0., 0.001]])},
    {'V': np.array([[8.15162, 0.281, 0., 0.],
                     [0.03028, 0., 0.306, 0.],
                     [-0.3251, 0., 0., 0.001]])},
    {'V': np.array([[8.15162, 0.281, 0., 0.],
                     [0.03028, 0., 0.306, 0.],
                     [-0.3251, 0., 0., 0.001]])},
    
    
    
    {'V': np.array([[8.92688, 0.281, 0., 0.],
                     [-0.96451, 0., 0.306, 0.],
                     [2.27312, 0., 0., 0.001]])},
    
    {'V': np.array([[ 8.69161, 0.281, 0., 0.],
                     [-0.70423, 0., 0.306, 0.],
                     [-0.30493, 0., 0., 0.001]])},
    
    
    {'V': np.array([[8.45635, 0.281, 0., 0.],
                     [-0.44396, 0., 0.306, 0.],
                     [2.26496, 0., 0., 0.001]])}
   
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