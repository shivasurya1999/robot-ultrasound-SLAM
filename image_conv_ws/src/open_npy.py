#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde

# Load the .npy file for elastic fusion rpe results 
# data = np.load('error_array_bag5.npy') #straight line x-axis without phantom 
# data = np.load('error_array_bag6.npy') #2 circular loops without phantom high speed 
# data = np.load('error_array_bag7.npy') #2 circular loops with phantom high speed
# data = np.load('error_array_bag8.npy') #straight line x-axis with phantom  
# data = np.load('error_array_bag9.npy') #2 circular loops with phantom low speed 
# data = np.load('error_array_bag10.npy') #1 circular loop with phantom very low speed 
# data = np.load('error_array_bag11.npy') #straight line y-axis with phantom 
# data = np.load('error_array_bag12.npy') #straight line z-axis with phantom 

# Load the .npy file for orb slam rpe results 
# data = np.load('error_array_orb_bag5.npy') #straight line x-axis without phantom 
# data = np.load('error_array_orb_bag6.npy') #2 circular loops without phantom high speed 
# data = np.load('error_array_orb_bag7.npy') #2 circular loops with phantom high speed
# data = np.load('error_array_orb_bag8.npy') #straight line x-axis with phantom  
# data = np.load('error_array_orb_bag9.npy') #2 circular loops with phantom low speed 
# data = np.load('error_array_orb_bag10.npy') #1 circular loop with phantom very low speed 
# data = np.load('error_array_orb_bag11.npy') #straight line y-axis with phantom 
# data = np.load('error_array_orb_bag12.npy') #straight line z-axis with phantom 


#Scale corrected files 
#Elastic fusion 
# data = np.load('error_array_scef5.npy') #straight line x-axis without phantom 
# data = np.load('error_array_scef6.npy') #2 circular loops without phantom high speed 
# data = np.load('error_array_scef9.npy') #2 circular loops with phantom low speed 
# data = np.load('error_array_scef10.npy') #1 circular loop with phantom very low speed 

#ORB SLAM 
# data = np.load('error_array_scorb5.npy') #straight line x-axis without phantom 
# data = np.load('error_array_scorb6.npy') #2 circular loops without phantom high speed 
# data = np.load('error_array_scorb9.npy') #2 circular loops with phantom low speed 
data = np.load('error_array_scorb10.npy') #1 circular loop with phantom very low speed 


# Access the data
print("Error Data:")
print(data)


# Calculate the KDE as a smooth curve over the histogram
data_density = gaussian_kde(data)
x_vals = np.linspace(min(data), max(data), 1000) # Adjust the number of points for smoothness
kde_vals = data_density(x_vals)

# Plot the histogram
plt.hist(data, bins=20, color='skyblue', edgecolor='black', density=True) # Use density=True for normalization
plt.title('Error Distribution with Trendline')
plt.xlabel('Error')
plt.ylabel('Density')

# Plot the trendline (KDE)
plt.plot(x_vals, kde_vals, color='red', linestyle='-', linewidth=2)


# Analyze the error data
mean_error = np.mean(data)
median_error = np.median(data)
max_error = np.max(data)
min_error = np.min(data)
std_dev_error = np.std(data)


# Show mean and median
plt.axvline(mean_error, color='green', linestyle='dashed', linewidth=1)
plt.axvline(median_error, color='blue', linestyle='dashed', linewidth=1)

plt.grid(True)
plt.show()

# Analysis of the error data in millimeters
mean_error_mm = np.mean(data) * 1000
median_error_mm = np.median(data) * 1000
max_error_mm = np.max(data) * 1000
min_error_mm = np.min(data) * 1000
std_dev_error_mm = np.std(data) * 1000

print("\nAnalysis in Millimeters:")
print(f"Mean Error: {mean_error_mm:.2f} mm")
print(f"Median Error: {median_error_mm:.2f} mm")
print(f"Maximum Error: {max_error_mm:.2f} mm")
print(f"Minimum Error: {min_error_mm:.2f} mm")
print(f"Standard Deviation of Error: {std_dev_error_mm:.2f} mm")


# # Create a histogram of the error data
# plt.hist(data, bins=20, color='skyblue', edgecolor='black')
# plt.title('Error Distribution')
# plt.xlabel('Error')
# plt.ylabel('Frequency')
# plt.grid(True)
# plt.show()
