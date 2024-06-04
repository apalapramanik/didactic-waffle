from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data1 = [
   
    
    {'V': np.array([[0.02999, 0.281,   0.,      0.     ],
                    [0.70044, 0. ,     0.306,   0.     ],
                    [1.5472 , 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.04498, 0.281,   0.,      0.     ],
                    [1.05066, 0. ,     0.306,   0.     ],
                    [1.5568, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.05998, 0.281,   0.,      0.     ],
                    [1.40088, 0. ,     0.306,   0.     ],
                    [1.5664, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.07497, 0.281,   0.,      0.     ],
                    [1.7511, 0. ,     0.306,   0.     ],
                    [1.576, 0.,      0. ,     0.001  ]])}]
    
    
    
    
    data2= [
    
    {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
                    [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
                    [0.,      0.,      0.,      0.001  , 0.     ],
                    [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
                    [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
                    [0.,      0.,      0.,      0.001  , 0.     ],
                    [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
                    [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
                    [0.,      0.,      0.,      0.001  , 0.     ],
                    [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
                    [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
                    [0.,      0.,      0.,      0.001  , 0.     ],
                    [0. ,     0.  ,    0.,      0.  ,    0.001  ]])}
   
   
    ]

    points1 = []
    for item in data1:
        V = item['V']
        x = V[0][0]
        y = V[1][0]
        points1.append([x, y])

   
    points_array1 = np.array(points1)
    
    points2 = []
    for item in data2:
        V = item['V']
        x = V[0][0]
        y = V[1][0]
        points2.append([x, y])

   
    points_array2 = np.array(points2)

  
    
    p = []
    

    dir_mat = np.array([[1, 0], [0,1]])
    
   
    
    dir_vec = np.array([0, 0])
    
    for i in range(len(points_array1)):
        c = (np.expand_dims(points_array1[i], axis=0)).transpose()
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
        
    for i in range(len(points_array2)):
        c = (np.expand_dims(points_array2[i], axis=0)).transpose()
        # print(c.shape)
        std = np.array([1.79,1.79])
        v = np.diag(np.array([1.79,1.79]))
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