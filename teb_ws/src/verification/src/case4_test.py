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
    
    
    data = [
   
    
    {'V': np.array([[11.45896, 0.281, 0., 0.],
                     [-5.57957, 0., 0.306, 0.],
                     [ 1.33124, 0., 0., 0.001]])},
    {'V': np.array([[11.47503, 0.281, 0., 0.],
                     [-5.22817 , 0., 0.306, 0.],
                     [ 1.23432, 0., 0., 0.001]])},
    {'V': np.array([[11.4911, 0.281, 0., 0.],
                     [-4.87817, 0., 0.306, 0.],
                     [1.23432, 0., 0., 0.001]])},
    {'V': np.array([[11.50718, 0.281, 0., 0.],
                     [-4.52538, 0., 0.306, 0.],
                     [1.04048, 0., 0., 0.001]])},
   
    
    
    
    
    
    {'V': np.array([[11.4465, 0.281, 0.],
                     [-4.3339, 0., 0.306]
                    ])},
    
    {'V': np.array([[11.4465, 0.281, 0.],
                     [-4.3339, 0., 0.306]
                    ])},
    
    {'V': np.array([[11.4465, 0.281, 0.],
                     [-4.3339, 0., 0.306]
                    ])},
    
    
    {'V': np.array([[11.4465, 0.281, 0.],
                     [-4.3339, 0., 0.306]
                    ])},
   
    ]

    points = []
    for item in data:
        V = item['V']
        x = V[0][0]
        y = V[1][0]
        points.append([x, y])

   
    points_array = np.array(points)

    print(points_array)
    
    p1 = []
    p2 = []
    p = []
    

    dir_mat = np.array([[0.55, 0], [0, 0.55]])
    
   
    
    dir_vec = np.array([0, 0])
    
    for i in range(len(points_array[0:4])):
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
        p1.append(prob)
        p.append(prob)
        
    for i in range(4):
        X = np.array([11.446500, -4.333900])
        c = (np.expand_dims(X, axis=0)).transpose()
        
        
        v = np.diag(np.array([0.1,0.1]))
        V = np.concatenate([c, v], axis =1)
        # print(V)
        C = []
        d = []
        mu = np.ones(2)
        sig = np.diag(np.ones(2))
        lb = np.array([-4.5,-4.5])
        ub = np.array([4.5,4.5])
        prob = ProbStar(V, C, d, mu, sig, lb, ub)
        p2.append(prob)
        p.append(prob)
        
    for i in range(4):
        ps, cp = probstar_halfspace_intersection_2d(p2[i], p1[i])
        # p.append(ps)
        print(cp)
    plot_probstar(p, dir_mat=dir_mat, dir_vec=dir_vec)
        
    
    
if __name__ == "__main__":
    plot()