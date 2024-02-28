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


# def test_random_plot():
#     mu = np.random.rand(2,)
#     Sig = np.eye(2)
#     pred_lb = np.random.rand(2,)
#     pred_ub = pred_lb + 0.2
#     print('Testing plot_probstar method...')
#     S = ProbStar(mu, Sig, pred_lb, pred_ub)
#     print(S)
#     S1 = S.affineMap(A=np.random.rand(2,2))
#     # P = []
#     # P.append(S)
#     # P.append(S1)
#     plot_probstar(S)
    
# def test_plot():
#     mu_initial = np.array([11.45685789395808, -7.218021583352927])
#     std_initial_rob = np.array([0.281, 0.306])
#     sigma = np.diag(np.square(std_initial_rob))
#     ub = (mu_initial + std_initial_rob ) 
#     print(ub)
#     lb = (mu_initial - std_initial_rob )
#     print(lb)
    
#     P = ProbStar(mu_initial, sigma, lb, ub)
#     print(P)
#     plot_probstar(P, show=True)
    
import matplotlib.pyplot as plt

# Define the points
points = [
    (0.3012672611032632, 2.425864716072617),
    (0.29821860546134354, 2.4917074713587946),
    (0.29813571789506177, 2.4917914295298633),
    (0.2981295789003906, 2.4894381587961516),
    (0.29813160330160676, 2.4862847736147504),
    (0.29813230527562223, 2.483108698541511),
    (0.29813236, 2.46809),
    (0.29813230527562223, 2.483108698541511),
    (0.3443220710587546, 2.5889535545967672),
    (0.387107016239488, 2.5830822624227063),
    (0.4812979799623133, 2.5720595858723603),
    (0.6909295593736993, 2.561128956286761),
    (1.158454643575398, 2.5526616210366924),
    (0.3113627, 2.5454116),
    (1.158454643575398, 2.5526616210366924),
    (0.31155092946772556, 2.7404695547725386),
    (0.311549266311217, 2.7799952018476977),
    (0.3115484099919616, 2.8099330252540593),
    (0.31154797960908687, 2.8318112338495207),
    (0.31154776396111056, 2.8469533292526386)
]

# Separate x and y coordinates
x_coords, y_coords = zip(*points)

# Split the points into original and estimated
original_points = [(0.2886664, 2.4262047), (0.29813236, 2.46809), (0.3113627, 2.5454116)]
estimated_points = points[:-2]

# Separate x and y coordinates for original and estimated points
original_x, original_y = zip(*original_points)
estimated_x, estimated_y = zip(*estimated_points)

# Create a scatter plot
plt.scatter(estimated_x, estimated_y, color='green', label='Estimated Pose')
plt.scatter(original_x, original_y, color='red', label='Original Pose')

# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Estimated vs Original Poses')
plt.legend()

# Display the plot
plt.show()

   


# if __name__ == "__main__":
#     # test_random_plot()
#     test_plot()
