import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from StarV.set.star import Star
import pypoman
import warnings
from plot import plot_probstar, plot_star
import numpy as np
from math import cos, sin


def test_random_plot():
    mu = np.random.rand(2,)
    Sig = np.eye(2)
    pred_lb = np.random.rand(2,)
    pred_ub = pred_lb + 0.2
    print('Testing plot_probstar method...')
    S = ProbStar(mu, Sig, pred_lb, pred_ub)
    print(S)
    S1 = S.affineMap(A=np.random.rand(2,2))
    # P = []
    # P.append(S)
    # P.append(S1)
    plot_probstar(S)
    
def test_plot():
    mu_initial = np.array([11.45685789395808, -7.218021583352927])
    std_initial_rob = np.array([0.281, 0.306])
    sigma = np.diag(np.square(std_initial_rob))
    ub = (mu_initial + std_initial_rob ) / 2 
    print(ub)
    lb = (mu_initial - std_initial_rob  )/ 2
    print(lb)
    
    P = ProbStar(mu_initial, sigma, lb, ub)
    A = np.array([[1.0, 0.14], [0.0, 1.0]])
    S = P.affineMap(A)
    p =[]
    p.append(P)
    p.append(S)
    # print(P)
    # print(S)
    plot_probstar(p, show=True)
    
def test_3d_plot():
    
    # initial state probstar:
       
        mu_initial_human = np.array([11.47269567452632, -4.520384216572423, 1.62578])
        std_initial_human = np.array([0.281, 0.306, 0.01])
        
        # U_initial_huamn = np.array([velocity, heading_angle])
        sigma_human = np.diag(np.square(std_initial_human))
        lb_human = mu_initial_human - std_initial_human / 2
        ub_human = mu_initial_human + std_initial_human / 2
        
        initial_probstar_human = ProbStar(mu_initial_human, sigma_human, lb_human, ub_human)
        
        vel_rob = 0.26
        theta = 0.5
        # dt_rob = 0.14
        
        A_rob = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        
        b_rob = np.array([[0.87758, 0.0],
                          [0.47943, 0.0],
                          [0.0, 1.0]])
        
        u = np.array([0.50241, -0.10937])
        
        bu = np.matmul(b_rob, u).flatten()
        
        prob = initial_probstar_human.affineMap(A_rob, bu)
        prob2 = prob.affineMap(A_rob,bu)
        prob3 = prob2.affineMap(A_rob,bu)
        A_2d = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
        
        initial_2d = initial_probstar_human.affineMap(A_2d)
        
        # print(initial_2d.mu[0], initial_2d.mu[1])
        
        prob_2d = prob.affineMap(A_2d)
        # print(prob_2d.mu[0], prob_2d.mu[1])
        
        prob2_2d = prob2.affineMap(A_2d)
        # print(prob2_2d.mu[0], prob2_2d.mu[1])
        
        prob3_2d = prob3.affineMap(A_2d)
        print(prob3_2d.mu[0], prob3_2d.mu[1])
        
        print(initial_probstar_human)
        print(prob)
        print(prob2)
        print(prob3)
        
        p = []
        p.append(initial_2d)
        p.append(prob_2d)
        p.append(prob2_2d)
        p.append(prob3_2d)
        plot_probstar(p, show=True)
        
        

if __name__ == "__main__":
    # test_random_plot()
    test_3d_plot()
