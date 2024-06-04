from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star,plot_probstar_signal
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pypoman

def probstar_halfspace_intersection_2d(P1, P2):
    if isinstance(P1, ProbStar) and isinstance(P2, ProbStar):
        
        l, u = P2.getRanges()
        
        H = np.array([[1,0], 
                  [-1,0],
                  [0,1],
                  [0,-1]])
        
        g = np.array([u[0], -l[0], u[1], -l[1]])
        
        g = g.reshape(4,1)
        
        
        V = P1.V
        v_new = V[0:2, 1:3] #new basis vector
        c_new =  np.array([[V[0][0]], [V[1][0]]]) # new center
        V_new = np.concatenate([c_new, v_new], axis =1) #new combined V 
        C_new = np.matmul(H, v_new) #new constarint matrix
        d_new = g-np.matmul(H,c_new) 
        d_new = d_new.reshape(4,) #new constraint vector
        new_mu = P1.mu[0:2]
        new_sig = P1.Sig[0:2,0:2]
        new_pred_lb = P1.pred_lb[0:2]
        
        new_pred_ub = P1.pred_ub[0:2]
        
        intersection = ProbStar(V_new,C_new,d_new,new_mu, new_sig,new_pred_lb,new_pred_ub)
  
        collision_probability = intersection.estimateProbability()
     
        
        return intersection, collision_probability
    
    else:
        return("Error: Input is not a probstar")


def plot():
    
    
    data1 = [
   
    
    {'V': np.array([[0.0582,  0.281 ,  0.  ,    0.     ],
                    [0.70005, 0.,      0.306 ,  0.     ],
                    [1.56498, 0. ,     0.,     0.001  ]])},
    
    {'V': np.array( [[0.0873,  0.281 ,  0. ,     0.     ],
                    [1.05008, 0.,      0.306 ,  0.     ],
                    [1.60355, 0.,      0. ,     0.001  ]])},
    
    {'V': np.array([[0.1164,  0.281,   0.  ,    0.,     ],
                    [1.4001,  0.,      0.306 ,  0.     ],
                    [1.64212, 0.  ,    0.,      0.001  ]])},
    
    {'V': np.array([[0.14551, 0.281,   0.  ,   0.     ],
                    [1.75013, 0.  ,    0.306 ,  0.     ],
                    [1.68069, 0.,      0. ,     0.001  ]])}]
    
    
    
    
    data2= [
    
    {'V': np.array([[0.08077, 1.79 ,   0.  ,    0. ,     0.     ],
                    [2.86282, 0. ,     1.79 ,   0.,     0.     ],
                    [0. ,     0. ,     0. ,     0.001 ,  0.     ],
                    [0. ,     0.,      0.,      0.,     0.001  ]])},
    
    {'V': np.array([[0.08077, 1.79 ,   0.  ,    0. ,     0.     ],
                    [2.86282, 0. ,     1.79 ,   0.,     0.     ],
                    [0. ,     0. ,     0. ,     0.001 ,  0.     ],
                    [0. ,     0.,      0.,      0.,     0.001  ]])},
    {'V': np.array([[0.08077, 1.79 ,   0.  ,    0. ,     0.     ],
                    [2.86282, 0. ,     1.79 ,   0.,     0.     ],
                    [0. ,     0. ,     0. ,     0.001 ,  0.     ],
                    [0. ,     0.,      0.,      0.,     0.001  ]])},
    {'V': np.array([[0.08077, 1.79 ,   0.  ,    0. ,     0.     ],
                    [2.86282, 0. ,     1.79 ,   0.,     0.     ],
                    [0. ,     0. ,     0. ,     0.001 ,  0.     ],
                    [0. ,     0.,      0.,      0.,     0.001  ]])},
   
   
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

  
    
    p1 = []
    p2 = []
    overlap = []

    dir_mat = np.array([[1, 0.0], [0.0,1]])
    
   
    
    dir_vec = np.array([0, 0])
    
    for i in range(len(points_array1)):
        c = (np.expand_dims(points_array1[i], axis=0)).transpose()
        # print(c.shape)
       
        v = np.diag(np.array([0.281,0.306]))
        V = np.concatenate([c, v], axis =1)
        # print(V)
        C = []
        d = []
        mu = np.ones(2)
        sig = np.diag(np.ones(2))
        lb = np.array([-4.5,-4.5])
        ub = np.array([4.5,4.5])
        prob1 = ProbStar(V, C, d, mu, sig, lb, ub)
        p1.append(prob1)
        overlap.append(prob1)
        
    for i in range(len(points_array2)):
        c = (np.expand_dims(points_array2[i], axis=0)).transpose()
        # print(c.shape)
        
        v = np.diag(np.array([0.5,0.5]))
        V = np.concatenate([c, v], axis =1)
        # print(V)
        C = []
        d = []
        mu = np.ones(2)
        sig = np.diag(np.ones(2))
        lb = np.array([-4.5,-4.5])
        ub = np.array([4.5,4.5])
        prob2 = ProbStar(V, C, d, mu, sig, lb, ub)
        p2.append(prob2)
        overlap.append(prob2)
    # plot_probstar(p, dir_mat=dir_mat, dir_vec=dir_vec)
   
    
    # for i in range(len(points_array2)):
    #     p, cp = probstar_halfspace_intersection_2d(p1[i], p2[i])
    #     overlap.append(p)
    #     print(cp)
        
    plot_probstar(overlap,dir_mat=dir_mat)
        
        
        
    
    
if __name__ == "__main__":
    plot()