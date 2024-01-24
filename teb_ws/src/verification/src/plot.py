import matplotlib.pyplot as plt
import numpy as np

def plot_from_files():
    # Load data from files
    states_history = np.loadtxt('states.txt')
    next_state = np.loadtxt('next_state.txt')    
    
    # Plotting
    plt.figure(figsize=(10, 6))

    # Plot states_history
    plt.plot(range(len(states_history)), states_history, marker='o', linestyle='-', color='b', label='States History')

    # Plot next_state
    plt.plot([len(states_history) - 1, len(states_history)], [states_history[-1], next_state], marker='o', linestyle='-', color='r', label='Next State')

    plt.title('States History and Next State')
    plt.xlabel('Index')
    plt.ylabel('State Value')
    plt.legend()

    plt.show()
    
if __name__ == '__main__':
    plot_from_files()
