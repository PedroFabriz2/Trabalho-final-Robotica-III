import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

plot_best_fit = True

csv_file = "arm_workspace_points.csv"

x_points = []
y_points = []
z_points = []

with open(csv_file, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  
    for row in reader:
        x, y, z = map(float, row)
        x_points.append(x)
        y_points.append(y)
        z_points.append(z)

if not plot_best_fit:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_points, y_points, z_points, c='b', marker='*', edgecolors='pink')

    plt.show()

else:
    def sphere_residuals(params, x):
        x0, y0, z0, r = params
        return (x[0] - x0)**2 + (x[1] - y0)**2 + (x[2] - z0)**2 - r**2

    initial_params = [0.0, 0.0, 0.0, 1.0]

    result = least_squares(sphere_residuals, initial_params, args=([x_points, y_points, z_points],))
    best_fit_params = result.x

    x0, y0, z0, radius = best_fit_params

    print(f"The best-fit sphere has center ({x0}, {y0}, {z0}) and radius {radius}")

    center = (x0, y0, z0)

    phi = np.linspace(0, np.pi, 20)
    theta = np.linspace(0, 2 * np.pi, 20)
    phi, theta = np.meshgrid(phi, theta)
    x = center[0] + radius * np.sin(phi) * np.cos(theta)
    y = center[1] + radius * np.sin(phi) * np.sin(theta)
    z = center[2] + radius * np.cos(phi)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(x, y, z, rstride=1, cstride=1, color='orangered', edgecolors='k', lw=0.2)
    ax.scatter(x_points, y_points, z_points, c='b', marker='*', edgecolors='pink')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


center = (x0, y0, z0)

phi = np.linspace(0 + np.pi/4, np.pi - np.pi/4, 20)
theta = np.linspace(0, 2 * np.pi, 20)
phi, theta = np.meshgrid(phi, theta)

x = center[0] + radius * np.sin(phi) * np.cos(theta)
y = center[1] + radius * np.sin(phi) * np.sin(theta)
z = center[2] + radius * np.cos(phi)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x[0:5], y[0:5], z[0:5], c='b', marker='*', edgecolors='black')

csv_file = "trajectory_points.csv"
points = [x[0:5], y[0:5], z[0:5]]

print(y[0])

print(points[1][0])

fields = ['X', 'Y', 'Z']
   
# data rows of csv file
rows = [ [x[0], y[0], z[0]],
         [x[1], y[1], z[1]],
         [x[2], y[2], z[2]],
         [x[3], y[3], z[3]],
         [x[4], y[4], z[4]]]
 
with open(csv_file, 'w') as f:
     
    # using csv.writer method from CSV package
    write = csv.writer(f)
     
    write.writerow(fields)
    write.writerows(rows)

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()



