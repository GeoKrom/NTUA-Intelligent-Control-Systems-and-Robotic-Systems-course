import pandas as pd
import matplotlib.pyplot as plt

CSV_FILENAME = "output_MRAC.csv"
THETA_1_POS = 11
THETA_2_POS = 12
THETA_3_POS = 13 

df = pd.read_csv(CSV_FILENAME)
filtered_df = df[ df["time_arduino"] != 0]

# Save values 
time = filtered_df["time_python"].tolist()
start_time = time[0]
actual_time = [t - start_time for t in time]
h1 = filtered_df["h1"].tolist()
h2 = filtered_df["h2"].tolist()
state = [state_str.split(",") for state_str in filtered_df["state"].tolist()]
theta1 = [float(x[THETA_1_POS].strip().replace("'", "")) for x in state]
theta2 = [float(x[THETA_2_POS].strip().replace("'", "")) for x in state]
theta3 = [float(x[THETA_1_POS].strip().replace("'", "")) for x in state]

# Plots 
_, axes_h = plt.subplots(nrows=2, ncols=1, figsize=(8, 6))
axes_h[0].plot(actual_time, h1, color='red')
axes_h[0].set_xlabel(r'$time [sec]$')
axes_h[0].set_ylabel(r'$h_1 [m]$')
axes_h[0].grid()
axes_h[0].grid(b=True, which='minor', linestyle='--')

axes_h[1].plot(actual_time, h2, color='red')
axes_h[1].set_xlabel(r'$time [sec]$')
axes_h[1].set_ylabel(r'$h_2 [m]$')
axes_h[1].grid()
axes_h[1].grid(b=True, which='minor', linestyle='--')

_, axes_theta= plt.subplots(nrows=3, ncols=1, figsize=(8, 6))
axes_theta[0].plot(actual_time, theta1, color='blue')
axes_theta[0].set_xlabel(r'$time [sec]$')
axes_theta[0].set_ylabel(r'$\theta_1$')
axes_theta[0].grid()
axes_theta[0].grid(b=True, which='minor', linestyle='--')

axes_theta[1].plot(actual_time, theta2, color='blue')
axes_theta[1].set_xlabel(r'$time [sec]$')
axes_theta[1].set_ylabel(r'$\theta_2$')
axes_theta[1].grid()
axes_theta[1].grid(b=True, which='minor', linestyle='--')

axes_theta[2].plot(actual_time, theta3, color='blue')
axes_theta[2].set_xlabel(r'$time [sec]$')
axes_theta[2].set_ylabel(r'$\theta_3$')
axes_theta[2].grid()
axes_theta[2].grid(b=True, which='minor', linestyle='--')

plt.tight_layout()
plt.show()
