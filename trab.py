import numpy as np
from roboticstoolbox import DHLink, SerialLink
from spatialmath import SE3
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import csv

show_robot = False

H = 1.75
L1 = 0.186*H
L2 = 0.146*H
L3 = 0.054*H
a0 = 0.2
d0 = 0.2

num_points = 4

link0_params = [0, a0, d0, 0]
link1_params = [np.deg2rad(-90), 0, 0, None]
link2_params = [np.deg2rad(90), 0, 0, None]
link3_params = [0, L1, 0, None]
link4_params = [np.deg2rad(90), 0, 0, None]
link5_params = [np.deg2rad(90), 0, L2, None]
link6_params = [np.deg2rad(90), 0, 0, None]
link7_params = [0, L3, 0, None]

dh_params = [
    link0_params,
    link1_params,
    link2_params,
    link3_params,
    link4_params,
    link5_params,
    link6_params,
    link7_params,
]

links = []
for params in dh_params:
    alpha, a, d, theta = params
    link = DHLink(a=a, alpha=alpha, d=d)
    links.append(link)

robot = SerialLink(links, name="Human Arm")

if show_robot:
    joint_angles = np.array([0, 0, np.deg2rad(-50), np.deg2rad(-50), np.deg2rad(10), 0, np.deg2rad(41.67), np.deg2rad(-50)]) 
    # joint_angles = np.array([0, np.deg2rad(0), np.deg2rad(0), np.deg2rad(44.44), 0, np.deg2rad(41.67), np.deg2rad(0), 0]) 
    robot.plot(joint_angles, block=True)
else:
    theta1_range = np.linspace(0, np.deg2rad(100), num_points)  
    theta2_range = np.linspace(np.deg2rad(-50), np.deg2rad(77.78), num_points)  
    theta3_range = np.linspace(np.deg2rad(-50), np.deg2rad(94.44), num_points)  
    theta4_range = np.linspace(np.deg2rad(44.44), np.deg2rad(130.56), num_points)  
    theta5_range = np.linspace(np.deg2rad(0), np.deg2rad(100), num_points)  
    theta6_range = np.linspace(np.deg2rad(41.67), np.deg2rad(72.22), num_points)  
    theta7_range = np.linspace(np.deg2rad(-50), np.deg2rad(38.89), num_points)  


    x_positions = []
    y_positions = []
    z_positions = []

    for theta1 in theta1_range:
        for theta2 in theta2_range:
            for theta3 in theta3_range:
                for theta4 in theta4_range:
                    for theta5 in theta5_range:
                        for theta6 in theta6_range:
                            for theta7 in theta7_range:

                                joint_angles = np.array([0, theta1, theta2, theta3, theta4, theta5, theta6, theta7])

                                end_effector_pose = robot.fkine(joint_angles)
                                position = end_effector_pose.t

                                x_positions.append(position[0])
                                y_positions.append(position[1])
                                z_positions.append(position[2])


    csv_file = "arm_workspace_points.csv"
    points = list(zip(x_positions, y_positions, z_positions))


    with open(csv_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["X", "Y", "Z"])  
        writer.writerows(points)                          


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x_positions, y_positions, z_positions, c='b', marker='*', edgecolors='r')
    ax.scatter(0,0,0, c='b', marker='o', edgecolors='b')
    plt.show()
