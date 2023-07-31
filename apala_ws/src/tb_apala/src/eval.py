import numpy as np
from sklearn.metrics import mean_squared_error

def find_min_value_in_file(file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            # Convert lines to a list of floating-point numbers
            values = [float(line.strip()) for line in lines]

            if not values:
                raise ValueError("The file is empty or does not contain valid numeric values.")

            min_value = min(values)
            return min_value

    except FileNotFoundError:
        print("Error: The file was not found.")
    except ValueError as ve:
        print(f"Error: {ve}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
 
    
def print_values_in_file(file_path):
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                value = line.strip()
                print("Time taken to complete goal:", value)

    except FileNotFoundError:
        print("Error: The file was not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


base_folder = "loc1h2"

#calculate minimum robustness of spec1:

file_path1 = f"{base_folder}/always_human1.txt"
min_value1 = find_min_value_in_file(file_path1)
if min_value1 is not None:
    print(f"The minimum robustness in spec 1: {min_value1}")
    
#calculate minimum robustness of spec2:

file_path2 = f"{base_folder}/implies_human1.txt"
min_value2 = find_min_value_in_file(file_path2)
if min_value2 is not None:
    print(f"The minimum robustness in spec 2: {min_value2}")
    
#calculate minimum robustness of spec3:

file_path3 =f"{base_folder}/until_human1.txt"
min_value3 = find_min_value_in_file(file_path3)
if min_value3 is not None:
    print(f"The minimum robustness in spec 3: {min_value3}")
    
#time taken to complete task    
file_path_to_read = f"{base_folder}/goal_time.txt"
print_values_in_file(file_path_to_read)

# calculate rmse:
original_points = np.loadtxt(f"{base_folder}/org_h1.txt", delimiter=",")
predicted_points_kf = np.loadtxt(f"{base_folder}/pred_kf_h1.txt", delimiter=",")

x = [p[0] for p in original_points[:]]
y = [p[1] for p in original_points[:]]

w_kf = [m[0] for m in predicted_points_kf[:]]
z_kf = [m[1] for m in predicted_points_kf[:]]

org_cols = original_points[:,:2]
pred_cols_kf = predicted_points_kf[:,:2]

rmse_kf = np.sqrt(mean_squared_error(org_cols, pred_cols_kf))
print("Root mean squared error: ", rmse_kf)



