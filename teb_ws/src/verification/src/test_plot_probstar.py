import numpy as np
from StarV.set.probstar import ProbStar
from StarV.util.plot import probstar2polytope, plot_probstar

mu = np.random.rand(3,)
Sig = np.eye(3)
pred_lb = np.random.rand(3,)
pred_ub = pred_lb + 0.2
print('Testing probstar2Polytope method...')
S = ProbStar(mu, Sig, pred_lb, pred_ub)

P = probstar2polytope(S)
print('Polytope = {}'.format(P))