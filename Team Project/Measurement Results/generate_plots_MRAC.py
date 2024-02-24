import pandas as pd
import matplotlib.pyplot as plt

CSV_FILENAME = "teamB/MRAC_Output.csv"
PARAM_1_POS = 12
PARAM_2_POS = 13
PARAM_3_POS = 14

df = pd.read_csv(CSV_FILENAME)
filtered_df = df[df["time_arduino"] != 0]

# Save values
time = filtered_df["time_python"].tolist()
start_time = time[0]
actual_time = [t - start_time for t in time]
h1 = [x/1000 for x in filtered_df["h1"].tolist()]
h2 = [x/1000 for x in filtered_df["h2"].tolist()]

OFFSET = 0.015  # TODO: Check if you need them
state = [state_str.split(",") for state_str in filtered_df["state"].tolist()]
xm = [float(x[10].strip().replace("'", ""))/1000 + OFFSET for x in state]
Kx1 = [float(x[PARAM_1_POS].strip().replace("'", ""))/1000 for x in state]
Kx2 = [float(x[PARAM_2_POS].strip().replace("'", ""))/1000 for x in state]
Kr = [float(x[PARAM_3_POS].strip().replace("'", ""))/1000 for x in state]

# Plots
_, axes_h = plt.subplots(nrows=2, ncols=1, figsize=(8, 6))
axes_h[0].plot(actual_time, h1, color='red')
axes_h[0].set_xlabel(r'$time [sec]$')
axes_h[0].set_ylabel(r'$h_1 [m]$')
axes_h[0].grid()
axes_h[0].grid(b=True, which='minor', linestyle='--')

axes_h[1].plot(actual_time, h2, color='red', label=r'$h_2$')
axes_h[1].plot(actual_time, xm, color='blue', linestyle="--", label=r'$x_m$')
axes_h[1].set_xlabel(r'$time [sec]$')
axes_h[1].set_ylabel(r'$h_2 [m]$')
axes_h[1].legend(loc="lower right")
axes_h[1].grid()
axes_h[1].grid(b=True, which='minor', linestyle='--')

_, axes_theta = plt.subplots(nrows=3, ncols=1, figsize=(8, 6))
axes_theta[0].plot(actual_time, Kx1, color='blue')
axes_theta[0].set_xlabel(r'$time [sec]$')
axes_theta[0].set_ylabel(r'$K_{x_1}$')
axes_theta[0].grid()
axes_theta[0].grid(b=True, which='minor', linestyle='--')

axes_theta[1].plot(actual_time, Kx2, color='blue')
axes_theta[1].set_xlabel(r'$time [sec]$')
axes_theta[1].set_ylabel(r'$K_{x_2}$')
axes_theta[1].grid()
axes_theta[1].grid(b=True, which='minor', linestyle='--')

axes_theta[2].plot(actual_time, Kr, color='blue')
axes_theta[2].set_xlabel(r'$time [sec]$')
axes_theta[2].set_ylabel(r'$K_r$')
axes_theta[2].grid()
axes_theta[2].grid(b=True, which='minor', linestyle='--')

plt.tight_layout()
plt.show()
