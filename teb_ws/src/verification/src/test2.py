import numpy as np
import matplotlib.pyplot as plt

def normal_distribution(mean, std):
  x = np.linspace(-3*std, 3*std, 100)
  y = np.exp(-(x - mean)**2 / (2 * std**2)) / (std * np.sqrt(2 * np.pi))
  return x, y

mean =  11.45685789395808
std = 5.868065846169667e-07
x, y = normal_distribution(mean, std)

plt.plot(x, y)
plt.xlabel('x')
plt.ylabel('y')
plt.title('Normal distribution with mean {} and standard deviation {}'.format(mean, std))
plt.show()