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
    ub = (mu_initial + std_initial_rob ) 
    print(ub)
    lb = (mu_initial - std_initial_rob )
    print(lb)
    
    P = ProbStar(mu_initial, sigma, lb, ub)
    print(P)
    plot_probstar(P, show=True)
   


if __name__ == "__main__":
    # test_random_plot()
    test_plot()
