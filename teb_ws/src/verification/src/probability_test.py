from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star
from StarV.set.star import Star
import numpy as np
from math import cos, sin

import glpk
import polytope as pc


def test_prob():
    
    A_2d= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
    
    
    ############# initial probstar 1 #####################################
    
    vel = 0.26
    omega = 0.2
    theta=1.6257967346611615   
    X = np.array([11.45685789395808,-7.218021583352927,1.6257967346611615])
    
    
    U = np.array([vel, omega]) 
    mu_initial_rob = X
    std_initial_rob = np.array([0.281, 0.306, 0.001]) 
    sigma_rob = np.diag(np.square(std_initial_rob))
    U_initial_rob = U      
    lb_rob = mu_initial_rob - std_initial_rob / 2
    ub_rob = mu_initial_rob + std_initial_rob / 2
    
    init_probstar_rob = ProbStar(mu_initial_rob, sigma_rob, lb_rob, ub_rob)
    
    
    
    # init_2d = init_probstar_rob.affineMap(A_2d) 
    
    # l1,u1 = init_2d.getRanges()
    # print("first:",l1,u1)  
    
    ##################### affine map init probstar 1 ######################################
    
    
    A_rob= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob = np.array([[cos(theta)*dtm, 0.0],
                            [sin(theta)*dtm, 0.0],
                            [0.0, 1.0]])
    
    bu = np.matmul(b_rob, U).flatten() 
    
    
    
    next = init_probstar_rob.affineMap(A_rob,bu)
    
    next_2d = next.affineMap(A_2d)
    l2,u2 = next_2d.getRanges()
    print("first_affmap:",l2,u2)

    
   
    
    #######################  case 1: #############################################################
    
    lb1 = np.array([11.31636, -7.1893])
    ub1= np.array([11.58735, -7.06502])

    A1 = np.array([
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ])

    b1 = np.array([ub1[0], -lb1[0], ub1[1], -lb1[1]])
    
    intersection1 = next_2d.addMultipleConstraints(A1,b1)
    
    # print(intersection1)    
    # l=[]
    # l.append(init_2d)
    # l.append(next_2d)
    # l.append(intersection)
    # plot_probstar(l)
    
    
    ##################  case 2 : ###################################################################
    
    
    #################### initial  probstar 2  ############################################
    
    vel2 = 0.26
    omega2 = 0.2
    theta2=1.6257967346611615   
    
    
    U2 = np.array([vel2, omega2])
        
    X2 = np.array([11.59685789395808,-7.288021583352927,1.6297967346611615])
    mu_initial_rob2 = X2
    std_initial_rob2 = np.array([0.381, 0.406, 0.005]) 
    sigma_rob2 = np.diag(np.square(std_initial_rob2))
       
    lb_rob2 = mu_initial_rob2 - std_initial_rob2 / 2
    ub_rob2 = mu_initial_rob2 + std_initial_rob2 / 2
    
 

    init_probstar_rob2 = ProbStar(mu_initial_rob2, sigma_rob2, lb_rob2, ub_rob2)
    
    init_2d2 = init_probstar_rob2.affineMap(A_2d) 
    l3,u3 = init_2d2.getRanges()
    # print("second:",l2,u2)  
    
    # h = []
    # h.append(init_2d)
    # h.append(init_2d2)
    # # plot_probstar(h)
    
    
    ########### init probstar 2 affine map ############################################
    
    A_rob2 = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm2 = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob2 = np.array([[cos(theta2)*dtm2, 0.0],
                            [sin(theta2)*dtm2, 0.0],
                            [0.0, 1.0]])
    
    bu2 = np.matmul(b_rob2, U2).flatten() 
    
    next2 = init_probstar_rob2.affineMap(A_rob2,bu2)
    
    next_2d2 = next2.affineMap(A_2d)
    l4,u4 = next_2d2.getRanges()
    print("second affmap:", l4, u4)
    
    # p = []
    # p.append(next_2d)
    # p.append(next_2d2)
    # plot_probstar(p)
    
    
    lb2 = np.array([11.39635, -7.1893])
    ub2 = np.array([11.58735, -6.8833])

    A2 = np.array([
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ])

    b2 = np.array([ub2[0], -lb2[0], ub2[1], -lb2[1]])
    
    intersection2 = next_2d.addMultipleConstraints(A2,b2)
    
    p = []
    p.append(next_2d)
    p.append(next_2d2)
    p.append(intersection2)
    
    plot_probstar(p)
    
    
    
    
    #####################  case 3: ##################################################################3
    
   


    
if __name__ == "__main__":
    test_prob()
    

    

