import pandas as pd
import matplotlib.pyplot as plt

CSV_FILENAME = "teamA/MIT_Output.csv"

df = pd.read_csv(CSV_FILENAME)

# Save values
time = df["time_python"].tolist()
start_time = time[0]
actual_time = [t - start_time for t in time]
h1 = [x/1000 for x in df["h1"].tolist()]
h2 = [x/1000 for x in df["h2"].tolist()]

#
h2_fixed = []
for h in h2:
    if (h < -0.05):
        h2_fixed.append(0.0)
    else:
        h2_fixed.append(h)


xm = [x/1000 for x in df["xm"].tolist()]
theta1 = [x/1000000 for x in df["theta1"].tolist()]
theta2 = [x/1000000 for x in df["theta2"].tolist()]
theta3 = [x/1000000 for x in df["theta3"].tolist()]

# Plots
_, axes_h = plt.subplots(nrows=2, ncols=1, figsize=(8, 6))
axes_h[0].plot(actual_time, h1, color='red')
axes_h[0].set_xlabel(r'$time [sec]$')
axes_h[0].set_ylabel(r'$h_1 [m]$')
axes_h[0].grid()
axes_h[0].grid(b=True, which='minor', linestyle='--')

axes_h[1].plot(actual_time, h2_fixed, color='red', label=r'$h_2$')
axes_h[1].plot(actual_time, xm, color='blue', linestyle="--", label=r'$x_m$')
axes_h[1].set_xlabel(r'$time [sec]$')
axes_h[1].set_ylabel(r'$h_2 [m]$')
axes_h[1].legend(loc="lower right")
axes_h[1].grid()
axes_h[1].grid(b=True, which='minor', linestyle='--')

_, axes_theta = plt.subplots(nrows=3, ncols=1, figsize=(8, 6))
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
