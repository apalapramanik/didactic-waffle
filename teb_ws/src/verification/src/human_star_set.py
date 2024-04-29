# points3 = [
#     (0.3742404282093048, 1.6616487503051758),
#     (0.3656119406223297, 1.670714259147644),
#     (0.3569834530353546, 1.6797797679901123),
#     (0.3483549654483795, 1.6888452768325806),
#     (0.3397264778614044, 1.6979107856750488)
# ]


from StarV.set.star import Star

from StarV.util.plot import plot_probstar, plot_star
import numpy as np
vm = 0.05
points = [
    (0.3742404282093048, 1.6616487503051758),
    (0.3656119406223297, 1.670714259147644),
    (0.3569834530353546, 1.6797797679901123),
    (0.3483549654483795, 1.6888452768325806),
    (0.3397264778614044, 1.6979107856750488)
]
p = []
for i in range(len(points)):
    x = np.asarray(points[i])
    c = (np.expand_dims(x, axis=0)).transpose()
    v = np.array([[vm, 0.0],
                [0.0, vm]])
    c_V = np.concatenate([c, v], axis =1)
    C =[]
    d =[]
    pred_lb = np.array([-1.0, -1.0])
    pred_ub = np.array([1.0, 1.0])
    s = Star(c_V, C, d, pred_lb, pred_ub)
    p.append(s)
    

plot_star(p)

