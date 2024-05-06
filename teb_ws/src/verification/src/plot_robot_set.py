from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data = [
    {'V': np.array([[11.52625, 0.281, 0., 0.],
                     [-4.32828, 0., 0.306, 0.],
                     [1.65607, 0., 0., 0.001]])},
    {'V': np.array([[11.53467, 0.281, 0., 0.],
                     [-4.0273, 0., 0.306, 0.],
                     [1.69382, 0., 0., 0.001]])},
    {'V': np.array([[11.54308, 0.281, 0., 0.],
                     [-3.72632, 0., 0.306, 0.],
                     [1.73157, 0., 0., 0.001]])},
    {'V': np.array([[11.5515, 0.281, 0., 0.],
                     [-3.42533, 0., 0.306, 0.],
                     [1.76932, 0., 0., 0.001]])},
    {'V': np.array([[11.55992, 0.281, 0., 0.],
                     [-3.12435, 0., 0.306, 0.],
                     [1.80706, 0., 0., 0.001]])},
    {'V': np.array([[11.56834, 0.281, 0., 0.],
                     [-2.82336, 0., 0.306, 0.],
                     [1.84481, 0., 0., 0.001]])}
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
    
    angle = np.radians(45)  # Convert 45 degrees to radians
    dir_mat = np.array([[1/np.cos(angle), np.sin(angle)],
                        [-np.sin(angle), 1/np.cos(angle)]])
    # dir_mat = np.array([[1, 0.5], [0,2]])
    
   
    
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