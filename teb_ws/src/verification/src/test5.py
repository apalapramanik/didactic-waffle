import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from StarV.set.star import Star
from StarV.util.plot import plot_probstar, plot_star
import numpy as np
from math import cos, sin

def test_plot():
    # ub = np.array([1, 1])
    # lb = -ub
    # S = Star(lb, ub)
    # print(S)
    # plot_star(S)
 
    # return
    X_initial = np.array([11.45685789395808,-7.218021583352927])#,1.6257967346611615])
    std_initial_rob = np.array([0.281, 0.306]) 
    sigma = np.diag(np.square(std_initial_rob))


    a = 0.5
        
    ub = X_initial + a * std_initial_rob + 5
    lb = X_initial - a * std_initial_rob 

    S = Star(lb, ub)
    print(S)
    print(S.estimateRanges())
    plot_star(S)
    return
    # plant = DLODE(self.A)
    # initial_probstar = ProbStar(X_initial, sigma, lb, ub)
    # # 
    # S = Star(lb, ub)
    # plot_star(S, show=True)
    initial_probstar = ProbStar(X_initial, sigma,  lb, ub)
    print(initial_probstar)
    plot_probstar(initial_probstar, show=True)

if __name__ == "__main__":
    test_plot()
