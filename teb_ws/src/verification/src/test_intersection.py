from StarV.set.star import Star
from StarV.set.probstar import ProbStar
from StarV.util.plot import plot_probstar, plot_star
import numpy as np



def star_set():
    lb = np.array([0,0])
    ub = np.array([2,2])
    S = Star(lb, ub)
    print(S)    
    plot_star(S)
    
    A=np.random.rand(2,2)
    print(A)
    S1 = S.affineMap(A)
    print(S1)
    
    p = []
    p.append(S)
    p.append(S1)
    plot_star(p)
        
def star_halfspace_intersection_2d(S1, S2):
    
    if isinstance(S1, Star) and isinstance(S2, Star):
        
        l, u = S2.getRanges()
        
        C = np.array([[1,0], 
                  [-1,0],
                  [0,1],
                  [0,-1]])
        
        d = np.array([u[0], -l[0], u[1], -l[1]])
        
        S3 = S1.addMultipleConstraints(C,d)
        
        p = []
        p.append(S1)
        p.append(S2)
        p.append(S3)
        plot_star(p)     
    else:
        print("Both inputs should be star!")
    
def star_set_int():
    lb1 = np.array([0,0])
    ub1 = np.array([2,2])
    S1 = Star(lb1, ub1)
 
    
    lb2 = np.array([-1,-1])
    ub2 = np.array([1,1])
    S2 = Star(lb2, ub2)
   
    star_halfspace_intersection_2d(S1, S2)
    
def star_set_affmap_int ():
    
    lb1 = np.array([0,0])
    ub1 = np.array([2,2])
    S1 = Star(lb1, ub1)
 
    
    lb2 = np.array([-1,-1])
    ub2 = np.array([1,1])
    S2 = Star(lb2, ub2)
    print(S2)
     
    A=np.random.rand(2,2)
    S3 = S2.affineMap(A)
   
    print(A)
    print(S2)
    
    p = []
    p.append(S2)
    p.append(S3)
    plot_star(p)
    
    
    star_halfspace_intersection_2d(S1, S3)
    
    
    
def probstar():  
    
    #DOESNT WORK!!!!
    
    # mu = np.random.rand(3,)
    # Sig = np.eye(3)
    # pred_lb = np.random.rand(3,)
    # pred_ub = pred_lb + 0.2
    # S = ProbStar(mu, Sig, pred_lb, pred_ub)
    # print(S.estimateProbability())
    # S1 = S.affineMap(A=np.eye(2,3))
    # print(S1.estimateProbability())
  
    # plot_probstar(S1)
    
    #WORKS!!
    
    # S = ProbStar.rand(3)
    # prob = S.estimateProbability()
    # print('prob = {}'.format(prob))
    # print(S)
    # V = np.random.rand(5, 4)
    # C = np.random.rand(4, 3)
    # d = np.random.rand(4,)
    # S2 = ProbStar(V, C, d, S.mu, S.Sig, S.pred_lb, S.pred_ub)
    # prob2 = S2.estimateProbability()
    # print(prob2)
    
    
    
    x = np.array([11.59,-7.28,1.62])
    
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
    print(p.estimateProbability())
    p1 = p.affineMap(A=np.eye(2,3))
    # print(mu.shape)
    plot_probstar(p1)
    
    
    
    
    
    
def probstar_halfspace_intersection_2d(P1, P2):
    if isinstance(P1, ProbStar) and isinstance(P2, ProbStar):
        
        l, u = P2.getRanges()
        
        C = np.array([[1,0], 
                  [-1,0],
                  [0,1],
                  [0,-1]])
        
        d = np.array([u[0], -l[0], u[1], -l[1]])
        
        P3 = P1.addMultipleConstraints(C,d)
    
    
if __name__ == "__main__":
    probstar()