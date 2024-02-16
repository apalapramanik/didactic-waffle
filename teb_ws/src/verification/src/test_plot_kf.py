import matplotlib.pyplot as plt

# Original and estimated positions (first 15 readings)
original_positions = [
 
    [0.6490681, 0.6680929], [0.6490365, 0.6680921], [0.64903045, 0.66808933],
    [0.6490228, 0.66808987]
]

estimated_positions = [
    [0.6490681171417236, 0.6680929064750671],
    [0.6490365266799927, 0.6680920720100403], [0.6490304470062256, 0.6680893301963806],
    [0.6490228176116943, 0.6680898666381836]
]

# Extract x and y coordinates for original and estimated positions
original_x = [pose[0] for pose in original_positions]
original_y = [pose[1] for pose in original_positions]

estimated_x = [pose[0] for pose in estimated_positions]
estimated_y = [pose[1] for pose in estimated_positions]


# Plot the positions as lines
plt.figure(figsize=(10, 6))
plt.plot(original_x, original_y, color='red', label='Original Pose')
plt.plot(estimated_x, estimated_y, color='green', label='Estimated Pose')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original and Estimated Positions')
plt.legend()
plt.grid(True)
plt.show()
