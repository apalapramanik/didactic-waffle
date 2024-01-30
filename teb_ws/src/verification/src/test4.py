import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pypoman
import warnings

def getVertices(I):
    """Get all vertices of a star"""
    
    assert isinstance(I, ProbStar) or isinstance(I, Star), 'error: input should be a ProbStar or a Star'
    if len(I.C) == 0:
        lb = I.pred_lb
        ub = I.pred_ub
        A = np.eye(I.nVars)
        C = np.vstack((A, -A))
        d = np.concatenate([ub, -lb])
    else:
        lb = I.pred_lb
        ub = I.pred_ub
        A = np.eye(I.dim)
        C1 = np.vstack((A, -A))
        d1 = np.concatenate([ub, -lb])
        C = np.vstack((I.C, C1))
        d = np.concatenate([I.d, d1])

    c = I.V[:, 0]
    V = I.V[:, 1:I.nVars + 1]

    proj = (V, c)
    ineq = (C, d)
    verts = pypoman.projection.project_polytope(proj, ineq)

    return verts

def plot_3D_Star(I, show=True):
    if I.dim != 3:
        raise Exception('Input set is not 3D star')

    verts = getVertices(I)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    try:
        pypoman.plot_polygon(verts, ax=ax)
    except Exception:
        warnings.warn(message='Potential floating-point error')

    if show:
        plt.show()

def plot_probstar_3D(I, dir_mat=None, dir_vec=None, show_prob=True, label=('$y_1$', '$y_2$', '$y_3$'), show=True):
    """Plot a star set in a specific direction
       y = dir_mat*x + dir_vec, x in I
    """

    if isinstance(I, ProbStar):
        I1 = I.affineMap(dir_mat, dir_vec)
        if I1.dim > 3:
            raise Exception('error: only 3D plot is supported')
        prob = I1.estimateProbability()
        plot_3D_Star(I, show=False)
        l, u = I1.getRanges()
        if show_prob:
            ax = plt.gca()
            ax.text(0.5 * (l[0] + u[0]), 0.5 * (l[1] + u[1]), 0.5 * (l[2] + u[2]), str(prob))
            ax.set_xlim(l[0], u[0])
            ax.set_ylim(l[1], u[1])
            ax.set_zlim(l[2], u[2])

    elif isinstance(I, list):
        L = []
        U = []
        for i in range(0, len(I)):
            I2 = I[i].affineMap(dir_mat, dir_vec)
            if I2.dim > 3:
                raise Exception('error: only 3D plot is supported')
            prob = I2.estimateProbability()
            plot_3D_Star(I2, show=False)
            l, u = I2.getRanges()
            if i == 0:
                L = l
                U = u
            else:
                L = np.vstack((L, l))
                U = np.vstack([U, u])
            if show_prob:
                ax = plt.gca()
                ax.text(0.5 * (l[0] + u[0]), 0.5 * (l[1] + u[1]), 0.5 * (l[2] + u[2]), str(prob))

        Lm = L.min(axis=0)
        Um = U.max(axis=0)
        ax = plt.gca()
        ax.set_xlim(Lm[0], Um[0])
        ax.set_ylim(Lm[1], Um[1])
        ax.set_zlim(Lm[2], Um[2])
    else:
        raise Exception('error: first input should be a ProbStar or a list of ProbStar')

    ax.set_xlabel(label[0], fontsize=13)
    ax.set_ylabel(label[1], fontsize=13)
    ax.set_zlabel(label[2], fontsize=13)
    plt.xticks(fontsize=13)
    plt.yticks(fontsize=13)

    if show:
        plt.show()
