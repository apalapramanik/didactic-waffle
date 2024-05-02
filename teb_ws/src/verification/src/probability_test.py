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
    
    print(next2.V)
    
    next_2d2 = next2.affineMap(A_2d)
    l4,u4 = next_2d2.getRanges()
    print("second affmap:", l4, u4)
    
    p = []
    p.append(next_2d)
    p.append(next_2d2)
    plot_probstar(p)
    
    
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
    
    # plot_probstar(p)
    
    
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
def test_prob2():
    
    A_2d= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
    
    
    ############# initial probstar 1 #####################################
    
    vel = 0.26
    omega = 0.2
    theta=1.6257967346611615   
    x = np.array([11.59,-7.28,1.62])
    
    
    
    
    
    
    U = np.array([vel, omega]) 
    mu_initial_rob = x
    std_initial_rob = np.array([0.281, 0.306, 0.001]) 
    sigma_rob = np.diag(np.square(std_initial_rob))
    U_initial_rob = U      
    lb_rob = mu_initial_rob - std_initial_rob / 2
    ub_rob = mu_initial_rob + std_initial_rob / 2
    
    init_probstar_rob = ProbStar(mu_initial_rob, sigma_rob, lb_rob, ub_rob)
    
    # c = (np.expand_dims(X, axis = 0)).transpose()
    # V = np.diag(std_initial_rob)
   
    
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
    # plot_probstar(next_2d2)
    l4,u4 = next_2d2.getRanges()
    # print("second affmap:", l4, u4)
    
    ################################################# half space intersection ############################################################
    
    # (H x V ) alpha <= g-H x c
    

    #find intersection bounds
    
    lb2 = np.array([11.39635, -7.1893])
    ub2 = np.array([11.58735, -6.8833])

    H = np.array([
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1]
    ])

    g = np.array([ub2[0], -lb2[0], ub2[1], -lb2[1]])
    g = g.reshape(4,1)
    
    V = next_2d.V  
    V_new = V[:,1:3]  
    c_new =  np.array([[V[0][0]], [V[1][0]]])     #center
    C_new = np.matmul(H, V_new) #constarint
    d_new = g-np.matmul(H,c_new)
    d_new = d_new.reshape(4,)   
    sig = np.diag(np.square(std_initial_rob[0:2]))   
    c_V = np.concatenate([c_new, V_new], axis =1)
   
    
    probstar_overlap = ProbStar(c_V, C_new, d_new, mu_initial_rob[0:2], sig, lb_rob[0:2], ub_rob[0:2])
    print(probstar_overlap)
    p =[]
    p.append(next_2d)
    p.append(probstar_overlap)
    plot_probstar(next_2d)
    print('Probability:',probstar_overlap.estimateProbability())
    print('Is it empty set? ', probstar_overlap.isEmptySet())
   
    
def test_prob3():
    
    A_2d= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
    
    
    ############# initial probstar 1 #####################################
    
    vel = 0.26
    omega = 0.2
    theta=1.6257967346611615   
    x = np.array([11.59,-7.28,1.62])
    
    U = np.array([vel, omega]) 
    deviation = np.array([0.281, 0.306, 0.001])    
    c = (np.expand_dims(x, axis=0)).transpose()
    V = np.diag(deviation)
    c_V = np.concatenate([c, V], axis =1)
    C = []
    d = []
    
    mu = np.zeros(x.shape[0])
    sigma = np.diag(np.ones(x.shape[0]))
    pred_lb = np.ones(x.shape[0]) * 4.5 * -1
    pred_ub = np.ones(x.shape[0]) * 4.5
    p = ProbStar(c_V, C, d, mu, sigma, pred_lb, pred_ub)
    print("first pstar prob:", p.estimateProbability())
    
   
    
   
   
    
    ##################### affine map init probstar 1 ######################################
    
    
    A_rob= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob = np.array([[cos(theta)*dtm, 0.0],
                            [sin(theta)*dtm, 0.0],
                            [0.0, 1.0]])
    
    bu = np.matmul(b_rob, U).flatten() 
    
    
    
    next = p.affineMap(A_rob,bu)
    print("next_prob:", next.estimateProbability())
    
    next_2d = next.affineMap(A_2d)
    l2,u2 = next_2d.getRanges()
    print("first_affmap bounds:",l2,u2)
    # plot_probstar(next_2d)

    
    
    #################### initial  probstar 2  ############################################
    
    vel2 = 0.26
    omega2 = 0.2
    theta2=1.6257967346611615   
    
    
    U2 = np.array([vel2, omega2])
        
    x2 = np.array([12.99,-7.78,2.92])
    
    deviation2 = np.array([0.281, 0.306, 0.001])    
    c2 = (np.expand_dims(x2, axis=0)).transpose()
    V2 = np.diag(deviation2)
    c_V2 = np.concatenate([c2, V2], axis =1)
    C2 = []
    d2 = []
    
    mu2 = np.zeros(x2.shape[0])
    sigma2 = np.diag(np.ones(x2.shape[0]))
    pred_lb2 = np.ones(x2.shape[0]) * 4.5 * -1
    pred_ub2 = np.ones(x2.shape[0]) * 4.5
    p2 = ProbStar(c_V2, C2, d2, mu2, sigma2, pred_lb2, pred_ub2)
    print("second pstar prob:", p2.estimateProbability())
    
    # init_2d2 = p2.affineMap(A_2d) 
    # l3,u3 = init_2d2.getRanges()
    # print("second:",l2,u2)  
    
    # plot_probstar(init_2d2)
    
    
   
    
    
    ########### init probstar 2 affine map ############################################
    
    A_rob2 = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm2 = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob2 = np.array([[cos(theta2)*dtm2, 0.0],
                            [sin(theta2)*dtm2, 0.0],
                            [0.0, 1.0]])
    
    bu2 = np.matmul(b_rob2, U2).flatten() 
    
    next2 = p2.affineMap(A_rob2,bu2)
    
    
    
    next_2d2 = next2.affineMap(A_2d)
    # plot_probstar(next_2d2)
    l4,u4 = next_2d2.getRanges()
    print("second affmap bounds:", l4, u4)
    # plot_probstar(next_2d2)
    
    # plot = []
    # plot.append(next_2d)
    # plot.append(next_2d2)
    # plot_probstar(plot)
    # plot_probstar(next_2d)
    # plot_probstar(next_2d2)
    
    
    ################################################# half space intersection ############################################################
    
    
    C = np.array([[1,0], 
                  [-1,0],
                  [0,1],
                  [0,-1]])
    
    d = np.array([u4[0], -l4[0], u4[1], -l4[1]])
    
    # overlap = ProbStar(next_2d.V, C, d,next_2d.mu, next_2d.Sig, next_2d.pred_lb, next_2d.pred_ub )
    # plot_probstar(overlap)
    s = next_2d.addMultipleConstraints(C,d)
    print(s.estimateProbability())
    
    # plot = []
    # plot.append(next_2d)
    # plot.append(next_2d2)
    # plot.append(s)
    # plot_probstar(plot)
    
    p = probstar_halfspace_intersection_2d(p, p2)
    print(p)
  
  
def test_prob4():  
    A_2d= np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0]])
        
        
    ############# initial probstar 1 #####################################
    
    vel = 0.26
    omega = 0.2
    theta=1.6257967346611615   
    x = np.array([11.59,-7.28,1.62])
    
    U = np.array([vel, omega]) 
    deviation = np.array([0.281, 0.306, 0.001])    
    c = (np.expand_dims(x, axis=0)).transpose()
    V = np.diag(deviation)
    c_V = np.concatenate([c, V], axis =1)
    C = []
    d = []
    
    mu = np.zeros(x.shape[0])
    sigma = np.diag(np.ones(x.shape[0]))
    pred_lb = np.ones(x.shape[0]) * 4.5 * -1
    pred_ub = np.ones(x.shape[0]) * 4.5
    p = ProbStar(c_V, C, d, mu, sigma, pred_lb, pred_ub)
    print("first pstar prob:", p.estimateProbability())
    

    


    
    ##################### affine map init probstar 1 ######################################
    
    
    A_rob= np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob = np.array([[cos(theta)*dtm, 0.0],
                            [sin(theta)*dtm, 0.0],
                            [0.0, 1.0]])
    
    bu = np.matmul(b_rob, U).flatten() 
    
    p_set1 = []
    for i in range(5):
        next = p.affineMap(A_rob,bu)  
        next_2d = next.affineMap(A_2d)
        p_set1.append(next_2d)
        

    
    
    #################### initial  probstar 2  ############################################
    
    vel2 = 0.26
    omega2 = 0.2
    theta2=1.6257967346611615   
    
    
    U2 = np.array([vel2, omega2])
        
    x2 = np.array([12.99,-7.78,2.92])
    
    deviation2 = np.array([0.281, 0.306, 0.001])    
    c2 = (np.expand_dims(x2, axis=0)).transpose()
    V2 = np.diag(deviation2)
    c_V2 = np.concatenate([c2, V2], axis =1)
    C2 = []
    d2 = []
    
    mu2 = np.zeros(x2.shape[0])
    sigma2 = np.diag(np.ones(x2.shape[0]))
    pred_lb2 = np.ones(x2.shape[0]) * 4.5 * -1
    pred_ub2 = np.ones(x2.shape[0]) * 4.5
    p2 = ProbStar(c_V2, C2, d2, mu2, sigma2, pred_lb2, pred_ub2)
   
    

    
    

    
    
    ########### init probstar 2 affine map ############################################
    
    A_rob2 = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    
    dtm2 = 0.7 #odom time period = 0.03 / no of obs
    
    b_rob2 = np.array([[cos(theta2)*dtm2, 0.0],
                            [sin(theta2)*dtm2, 0.0],
                            [0.0, 1.0]])
    
    bu2 = np.matmul(b_rob2, U2).flatten() 
    p_set2 =[]
    for i in range(5):
    
        next2 = p2.affineMap(A_rob2,bu2)
        next_2d2 = next2.affineMap(A_2d)
        # p_set1.append(next_2d2)
        
    p_total = []
    # p_total.append(p_set1)
    # p_total.append(p_set2)
    # for i in range(5):
    #     int, prob = probstar_halfspace_intersection_2d (p_set1[i], p_set2[i])
    #     p_total.append(int)
    plot_probstar(p_set1)
        
        
    
 
    
    ################################################# half space intersection ############################################################
    
    
    
    
    
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
    
    
   
    
   


    
if __name__ == "__main__":
    test_prob4()
    

    

