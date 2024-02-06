import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from StarV.set.star import Star
import pypoman
import warnings
from StarV.util.plot import plot_probstar, plot_star
import numpy as np
from math import cos, sin

def test_plot():
 
    mu_initial = np.array([11.45685789395808,-7.218021583352927])#,1.6257967346611615])
    std_initial_rob = np.array([0.281, 0.306]) 
    sigma = np.diag(np.square(std_initial_rob))
    ub = (mu_initial + std_initial_rob ) / 2
    lb = (mu_initial - std_initial_rob ) /2

    S = Star(lb, ub)
    print(S)
    plot_star(S)
    
    P = ProbStar(mu_initial, sigma, lb, ub)
    plot_probstar(P, show=True)
    print(P)
    return



 

if __name__ == "__main__":
    test_plot()
