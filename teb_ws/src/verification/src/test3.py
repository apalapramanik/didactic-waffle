from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar
import numpy as np
from math import cos, sin


def test_plot():
    dt = 0.25 #model_dt = 0.25/10       
    vel = 0.26
    theta=1.6257967346611615
    
    A = np.array([[1.0, 0.0, -1 * vel*cos(theta)*dt],
                 [0.0, 1.0, -1 * vel*sin(theta)*dt],
                 [0.0, 0.0, 1.0]])
    
    
    
    X_initial = np.array([11.45685789395808,-7.218021583352927,1.6257967346611615])
    std_initial = np.array([0.5, 0.5, 0.5])
    sigma = np.diag(np.ones(X_initial.shape[0]))
    # U_initial = self.U
    
    a = 2.0
        
    ub = X_initial + a * std_initial
    lb = X_initial - a * std_initial
    
    # plant = DLODE(self.A)
    initial_probstar = ProbStar(X_initial, sigma, lb, ub)
    print(initial_probstar.dim)
    p = initial_probstar.affineMap(A)
    l = []
    # l.append(initial_probstar)
    l.append(p)
    plot_probstar(p)
    
def test_plot_rand():
    mu = np.random.rand(2,)
    Sig = np.eye(2)
    pred_lb = np.random.rand(2,)
    pred_ub = pred_lb + 0.2
    print('Testing plot_probstar method...')
    S = ProbStar(mu, Sig, pred_lb, pred_ub)
    S1 = S.affineMap(A=np.random.rand(2,2))
    P = []
    P.append(S)
    P.append(S1)
    plot_probstar(P)
   
    
    
if __name__ == "__main__":
    test_plot_rand()
    

