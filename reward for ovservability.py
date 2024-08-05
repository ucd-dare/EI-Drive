import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d

def read_data(file_path):
    with open(file_path, 'r') as file:
        data = [int(line.strip()) for line in file]
    return data

# Read data from the four files
file1_data = read_data('coop4_coop_SERVER.txt')
file2_data = read_data('coop4_coop_YOLO.txt')
file3_data = read_data('coop4_nocoop_SERVER.txt')
file4_data = read_data('coop4_nocoop_YOLO.txt')

# Generate ticks (x-axis values)
ticks = np.array(range(1, 401))

# Function to smooth data using Gaussian filter and calculate standard deviation
# def smooth_data_with_std(data, sigma=3):
#     smoothed_data = gaussian_filter1d(data, sigma=sigma)
#     std_dev = np.std(data - smoothed_data)
#     lower_bound = smoothed_data - std_dev
#     upper_bound = smoothed_data + std_dev
#     return smoothed_data, lower_bound, upper_bound

def smooth_data_with_std(data, window_size=10):
    cumsum_vec = np.cumsum(np.insert(data, 0, 0)) 
    smooth_data = (cumsum_vec[window_size:] - cumsum_vec[:-window_size]) / window_size
    smooth_data = np.concatenate(([smooth_data[0]] * (window_size-1), smooth_data))
    std_dev = np.std(data - smooth_data)
    lower_bound = smooth_data - std_dev
    upper_bound = smooth_data + std_dev
    return smooth_data, lower_bound, upper_bound

# Smooth the data and calculate the bounds
file1_data_smooth, file1_lower, file1_upper = smooth_data_with_std(np.array(file1_data))
file2_data_smooth, file2_lower, file2_upper = smooth_data_with_std(np.array(file2_data))
file3_data_smooth, file3_lower, file3_upper = smooth_data_with_std(np.array(file3_data))
file4_data_smooth, file4_lower, file4_upper = smooth_data_with_std(np.array(file4_data))

# Plot the data with customization
plt.figure(figsize=(12, 8))

# Define colors for consistency
colors = ['b', 'r']

light_blue = (3/255, 198/255, 252/255)
orange = (255/255, 128/255, 0/255)

# Plot each dataset with standard deviation shading
plt.fill_between(ticks, file1_lower, file1_upper, color=colors[0], alpha=0.1)
plt.plot(ticks, file1_data_smooth, label='Server coop', linestyle='-', color=colors[0], linewidth=3)

plt.fill_between(ticks, file3_lower, file3_upper, color=light_blue, alpha=0.1)
plt.plot(ticks, file3_data_smooth, label='Server nocoop', linestyle='-', color=light_blue, linewidth=3)


plt.fill_between(ticks, file2_lower, file2_upper, color=colors[1], alpha=0.1)
plt.plot(ticks, file2_data_smooth, label='YOLO coop', linestyle='-', color=colors[1], linewidth=3)

plt.fill_between(ticks, file4_lower, file4_upper, color=orange, alpha=0.1)
plt.plot(ticks, file4_data_smooth, label='YOLO nocoop', linestyle='-', color=orange, linewidth=3)

# Add title and labels with larger font size
plt.title('Comparison of the number of bounding boxes detected', fontsize=22)

fontsize = 18
plt.xlabel('Ticks', fontsize=fontsize)
plt.ylabel('BBX', fontsize=fontsize)

# Add legend with larger font size
plt.legend(fontsize=fontsize)
plt.xlim(0, 400)
plt.ylim(0, 20)

# Add grid for better readability
plt.grid(True)

# Adjust tick parameters for better readability
plt.xticks(fontsize=fontsize)
plt.yticks(fontsize=fontsize)

# Show the plot
plt.show()
