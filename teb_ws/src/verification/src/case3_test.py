from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman




def plot():
    
    
    data = [
   
    
    {'V': np.array([[0.03048, 0.281,   0.,      0.     ],
                    [0.11675, 0. ,     0.306,   0.     ],
                    [3.11046, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.03014, 0.281,   0.,      0.     ],
                    [0.11676, 0. ,     0.306,   0.     ],
                    [3.11045, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.02981, 0.281,   0.,      0.     ],
                    [0.11677, 0. ,     0.306,   0.     ],
                    [3.11045, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.02947, 0.281,   0.,      0.     ],
                    [0.11679, 0. ,     0.306,   0.     ],
                    [3.11045, 0.,      0. ,     0.001  ]])},
    
    
    
    
    
    
    # {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
    #                 [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
    #                 [0.,      0.,      0.,      0.001  , 0.     ],
    #                 [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    # {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
    #                 [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
    #                 [0.,      0.,      0.,      0.001  , 0.     ],
    #                 [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    # {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
    #                 [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
    #                 [0.,      0.,      0.,      0.001  , 0.     ],
    #                 [0. ,     0.  ,    0.,      0.  ,    0.001  ]])},
    
    # {'V': np.array([[0.26828, 1.79,    0.,      0. ,     0.     ],
    #                 [1.72292, 0.  ,    1.79 ,   0. ,     0.     ],
    #                 [0.,      0.,      0.,      0.001  , 0.     ],
    #                 [0. ,     0.  ,    0.,      0.  ,    0.001  ]])}
   
   
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