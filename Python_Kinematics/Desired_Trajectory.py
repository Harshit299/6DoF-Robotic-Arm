import numpy as np
import math
import matplotlib.pyplot as plt

# Link lengths
a1, a2, a3 = 7.5, 30.0, 7.5

# Link offsets
d1, d2, d3 = 30.0, 32.0, 24.0

# Matrix representing EE frame orientation wrt base frame in zero configuration
R_zero = np.matrix([
    [0,  0, 1],
    [0, -1, 0],
    [1,  0, 0]
])

# function to extract joint and corner coordinates for plotting the arm
def extract_coords(t1, t2, t3, t4, t5, t6):

    # DH Parameter Table
    PT = [[t1,                  (90/180)*np.pi,   a1, d1], 
          [t2 + (90/180)*np.pi, 0,                a2, 0],
          [t3,                  (90/180)*np.pi,   a3, 0],
          [t4,                  -(90/180)*np.pi,  0,  d2],
          [t5,                  (90/180)*np.pi,   0,  0],
          [t6,                  0,                0,  d3]]

    # Homogeneous Transformation Matrices
    H0_1 = np.matrix([[np.cos(PT[0][0]), -np.sin(PT[0][0])*np.cos(PT[0][1]),  np.sin(PT[0][0])*np.sin(PT[0][1]), (PT[0][2])*np.cos(PT[0][0])],
                      [np.sin(PT[0][0]),  np.cos(PT[0][0])*np.cos(PT[0][1]), -np.cos(PT[0][0])*np.sin(PT[0][1]), (PT[0][2])*np.sin(PT[0][0])],
                      [0,                 np.sin(PT[0][1]),                   np.cos(PT[0][1]),                  PT[0][3]],
                      [0,                 0,                                  0,                                 1]])

    H1_2 = np.matrix([[np.cos(PT[1][0]), -np.sin(PT[1][0])*np.cos(PT[1][1]),  np.sin(PT[1][0])*np.sin(PT[1][1]), (PT[1][2])*np.cos(PT[1][0])],
                      [np.sin(PT[1][0]),  np.cos(PT[1][0])*np.cos(PT[1][1]), -np.cos(PT[1][0])*np.sin(PT[1][1]), (PT[1][2])*np.sin(PT[1][0])],
                      [0,                 np.sin(PT[1][1]),                   np.cos(PT[1][1]),                  PT[1][3]],
                      [0,                 0,                                  0,                                 1]])

    H2_3 = np.matrix([[np.cos(PT[2][0]), -np.sin(PT[2][0])*np.cos(PT[2][1]),  np.sin(PT[2][0])*np.sin(PT[2][1]), (PT[2][2])*np.cos(PT[2][0])],
                      [np.sin(PT[2][0]),  np.cos(PT[2][0])*np.cos(PT[2][1]), -np.cos(PT[2][0])*np.sin(PT[2][1]), (PT[2][2])*np.sin(PT[2][0])],
                      [0,                 np.sin(PT[2][1]),                   np.cos(PT[2][1]),                  PT[2][3]],
                      [0,                 0,                                  0,                                 1]])

    H3_4 = np.matrix([[np.cos(PT[3][0]), -np.sin(PT[3][0])*np.cos(PT[3][1]),  np.sin(PT[3][0])*np.sin(PT[3][1]), (PT[3][2])*np.cos(PT[3][0])],
                      [np.sin(PT[3][0]),  np.cos(PT[3][0])*np.cos(PT[3][1]), -np.cos(PT[3][0])*np.sin(PT[3][1]), (PT[3][2])*np.sin(PT[3][0])],
                      [0,                 np.sin(PT[3][1]),                   np.cos(PT[3][1]),                  PT[3][3]],
                      [0,                 0,                                  0,                                 1]])
    
    H4_5 = np.matrix([[np.cos(PT[4][0]), -np.sin(PT[4][0])*np.cos(PT[4][1]),  np.sin(PT[4][0])*np.sin(PT[4][1]), (PT[4][2])*np.cos(PT[4][0])],
                      [np.sin(PT[4][0]),  np.cos(PT[4][0])*np.cos(PT[4][1]), -np.cos(PT[4][0])*np.sin(PT[4][1]), (PT[4][2])*np.sin(PT[4][0])],
                      [0,                 np.sin(PT[4][1]),                   np.cos(PT[4][1]),                  PT[4][3]],
                      [0,                 0,                                  0,                                 1]])

    H5_6 = np.matrix([[np.cos(PT[5][0]), -np.sin(PT[5][0])*np.cos(PT[5][1]),  np.sin(PT[5][0])*np.sin(PT[5][1]), (PT[5][2])*np.cos(PT[5][0])],
                      [np.sin(PT[5][0]),  np.cos(PT[5][0])*np.cos(PT[5][1]), -np.cos(PT[5][0])*np.sin(PT[5][1]), (PT[5][2])*np.sin(PT[5][0])],
                      [0,                 np.sin(PT[5][1]),                   np.cos(PT[5][1]),                  PT[5][3]],
                      [0,                 0,                                  0,                                 1]])

    # Global Transformation Matrices
    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)
    H0_4 = np.dot(H0_3, H3_4)
    H0_5 = np.dot(H0_4, H4_5)
    H0_6 = np.dot(H0_5, H5_6)

    p1 = np.matrix([[0],[0],[0]]) # base joint 1
    p2 = np.matrix([[0],[0],[d1]]) # corner 1
    p3 = H0_1[0:3, 3] # joint 2
    p4 = H0_2[0:3, 3] # joint 3
    p5 = H0_3[0:3, 3] # corner 2
    p7 = H0_4[0:3, 3] # joint 5
    p9 = H0_6[0:3, 3] # EE

    # p6 (joint 4): move 7.97 cm forward from corner 2 towards joint 5
    forearm_vector = p7 - p5
    forearm_unit_vector = forearm_vector / np.linalg.norm(forearm_vector)
    p6 = p5 + (forearm_unit_vector * 7.97)
    
    # p8 (joint 6): move 8 cm forward from joint 5 towards EE
    tool_vector = p9 - p7
    tool_unit_vector = tool_vector / np.linalg.norm(tool_vector)
    p8 = p7 + (tool_unit_vector * 8)
    
    # Extract the 3x3 Rotation Matrix for the End Effector (for drawing arrows)
    R0_6 = H0_6[0:3, 0:3]

    return p1, p2, p3, p4, p5, p6, p7, p8, p9, R0_6

''' 
Defining this function because T1, T2, T3 would be changed 
when wrist center reaches its correct positions, so new R0_3 would be needed'''
def new_R0_3(theta1, theta2, theta3):
    PT_new = [[theta1,                  (90/180)*np.pi,   a1, d1], 
              [theta2 + (90/180)*np.pi,        0,         a2,  0],
              [theta3,                  (90/180)*np.pi,   a3,  0]]
    
    H0_1_new = np.matrix([[np.cos(PT_new[0][0]), -np.sin(PT_new[0][0])*np.cos(PT_new[0][1]),  np.sin(PT_new[0][0])*np.sin(PT_new[0][1]), (PT_new[0][2])*np.cos(PT_new[0][0])],
                          [np.sin(PT_new[0][0]),  np.cos(PT_new[0][0])*np.cos(PT_new[0][1]), -np.cos(PT_new[0][0])*np.sin(PT_new[0][1]), (PT_new[0][2])*np.sin(PT_new[0][0])],
                          [0,                     np.sin(PT_new[0][1]),                       np.cos(PT_new[0][1]),                       PT_new[0][3]],
                          [0,                     0,                                          0,                                          1]])

    H1_2_new = np.matrix([[np.cos(PT_new[1][0]), -np.sin(PT_new[1][0])*np.cos(PT_new[1][1]),  np.sin(PT_new[1][0])*np.sin(PT_new[1][1]), (PT_new[1][2])*np.cos(PT_new[1][0])],
                          [np.sin(PT_new[1][0]),  np.cos(PT_new[1][0])*np.cos(PT_new[1][1]), -np.cos(PT_new[1][0])*np.sin(PT_new[1][1]), (PT_new[1][2])*np.sin(PT_new[1][0])],
                          [0,                     np.sin(PT_new[1][1]),                       np.cos(PT_new[1][1]),                       PT_new[1][3]],
                          [0,                     0,                                          0,                                          1]])

    H2_3_new = np.matrix([[np.cos(PT_new[2][0]), -np.sin(PT_new[2][0])*np.cos(PT_new[2][1]),  np.sin(PT_new[2][0])*np.sin(PT_new[2][1]), (PT_new[2][2])*np.cos(PT_new[2][0])],
                          [np.sin(PT_new[2][0]),  np.cos(PT_new[2][0])*np.cos(PT_new[2][1]), -np.cos(PT_new[2][0])*np.sin(PT_new[2][1]), (PT_new[2][2])*np.sin(PT_new[2][0])],
                          [0,                     np.sin(PT_new[2][1]),                       np.cos(PT_new[2][1]),                       PT_new[2][3]],
                          [0,                     0,                                          0,                                          1]])
    
    H0_3_new = H0_1_new @ H1_2_new @ H2_3_new

    return H0_3_new[0:3, 0:3]

# Function to convert RPY to Rotation Matrix for the current point in trajectory
def return_cur_orientation(roll, pitch, yaw):

    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    cur_orientation = (Rz @ Ry @ Rx) @ R_zero

    return cur_orientation

# IK function
def inverse_kinematics(target_pos, target_orientation):

    tool_offset = np.matrix([[0], [0], [d3]])
    wrist_pos = np.array(target_pos).reshape(3,1) - target_orientation @ tool_offset

    wrist_x_coord = float(wrist_pos[0, 0]) 
    wrist_y_coord = float(wrist_pos[1, 0]) 
    wrist_z_coord = float(wrist_pos[2, 0]) 

    r1 = math.sqrt(wrist_x_coord**2 + wrist_y_coord**2)
    theta1 = math.atan2(wrist_y_coord, wrist_x_coord)
    
    try:
        BC = r1 - a1
        KI = wrist_z_coord - d1
        RK = math.sqrt(a3**2 + d2**2)
        QK = math.sqrt(KI**2 + BC**2)

        beta = math.atan2(d2, a3)
        gamma = math.acos((RK**2 + a2**2 - QK**2)/(2*RK*a2))
        theta3 = beta - np.pi + gamma

        omega = math.atan2(KI, BC)
        phi1 = math.acos((a2**2 + QK**2 - RK**2)/(2*a2*QK))
        theta2 = phi1 - np.pi/2 + omega

        R0_3_new = new_R0_3(theta1=theta1, theta2=theta2, theta3=theta3)
        R3_6_new = R0_3_new.T @ target_orientation

        # safety check to prevent sudden flipping of wrist
        theta5 = math.acos(np.clip(R3_6_new[2, 2], -1.0, 1.0))
        if abs(math.sin(theta5)) > 1e-6:
            theta4 = math.atan2(R3_6_new[1, 2], R3_6_new[0, 2])
            theta6 = math.atan2(R3_6_new[2, 1], -R3_6_new[2, 0])
        else:
            theta4 = 0.0
            theta6 = math.atan2(-R3_6_new[1, 0], R3_6_new[0, 0])

        return theta1, theta2, theta3, theta4, theta5, theta6

    except ValueError:
        return None, None, None, None, None, None

# Function to implement logic for trajectory data generation
def generate_trajectory():

    # Ellipse parameters
    t_values = np.linspace(0, 2*np.pi, 150)

    a = 70    # ellipse semi-major axis
    b = 50     # ellipse semi-minor axis
    # center of ellipse
    xc = 50
    yc = 2
    zc = 10

    trajectory_data = []

    roll = np.deg2rad(-41)
    pitch = np.deg2rad(24)
    yaw = np.deg2rad(0)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])

    R_zero = np.matrix([
        [0,  0, 1],
        [0, -1, 0],
        [1,  0, 0]
    ])

    # EXTRINSIC rotation
    target_orientation = (Rz @ Ry @ Rx) @ R_zero

    for t in t_values:

        # generate an ellipse
        x = xc + a*np.cos(t)
        y = yc + b*np.sin(t)
        z = zc + a*np.cos(t)
        # z = zc

        target_pos = np.array([x, y, z])

        cur_angles = inverse_kinematics(target_pos, target_orientation)

        if cur_angles is not None and all(angle is not None for angle in cur_angles):
            trajectory_data.append((cur_angles, target_pos, target_orientation, (roll, pitch, yaw)))
        else:
            print(f"Skipping unreachable point at t={t:.3f}")

    return trajectory_data

# Function to visualize the trajectory using Matplotlib
def visualize_trajectory(trajectory_list):
    plt.ion() 
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    path_x, path_y, path_z = [], [], []
    limit = 125 # axes limit

    for angles, _, _, target_rpy in trajectory_list:
        ax.cla()

        th1, th2, th3, th4, th5, th6 = angles
        p1, p2, p3, p4, p5, p6, p7, p8, p9, R0_6 = extract_coords(th1, th2, th3, th4, th5, th6)
        
        # Update path using EE position
        path_x.append(p9[0])
        path_y.append(p9[1])
        path_z.append(p9[2])
        
        # Plot EE trace & waypoints
        ax.plot(path_x, path_y, path_z, 'r--', linewidth=1, label='Path', zorder=1)

        # Extract individual arrays for plotting the arms
        line_pts = [p1, p2, p3, p4, p5, p6, p7, p8, p9]
        xs = [p[0] for p in line_pts]
        ys = [p[1] for p in line_pts]
        zs = [p[2] for p in line_pts]
        
        # Plot the arms connecting everything
        ax.plot(xs, ys, zs, '-', color='dimgrey', linewidth=4, zorder=3)
        
        # Plot the specific colored dots for each joint and corner
        colors = ['blue', 'black', 'red', 'green', 'black', 'orange', 'cyan', 'magenta', 'purple']
        for pt, col in zip(line_pts, colors):
            ax.scatter(*pt, color=col, s=80, zorder=4, depthshade=False)
            
        # Draw the EE orientation arrows (Red=X, Green=Y, Blue=Z)
        R = np.asarray(R0_6)
        arrow_len = 35
        ax.quiver(p9[0], p9[1], p9[2], R[0,0], R[1,0], R[2,0], color='r', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        ax.quiver(p9[0], p9[1], p9[2], R[0,1], R[1,1], R[2,1], color='g', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        ax.quiver(p9[0], p9[1], p9[2], R[0,2], R[1,2], R[2,2], color='b', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(0, limit)
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        
        # Display positions and angles
        roll, pitch, yaw = target_rpy
        j1, j2, j3, j4, j5, j6 = np.degrees(angles)
        
        p9_vals = np.asarray(p9, dtype=float).reshape(-1)
        title_text = (f"Pos (XYZ): {p9_vals[0]:.1f}, {p9_vals[1]:.1f}, {p9_vals[2]:.1f} | RPY: {roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°\n"
                      f"Angles: J1:{j1:.1f}° J2:{j2:.1f}° J3:{j3:.1f}° J4:{j4:.1f}° J5:{j5:.1f}° J6:{j6:.1f}°")
        
        ax.set_title(title_text, fontsize=10, loc='left')
        
        plt.draw()
        plt.pause(0.01)

    plt.ioff()
    plt.show()

# Main function to run the trajectory generation and visualization
if __name__ == "__main__":

    full_trajectory = generate_trajectory()

    # Empty list to generate profiles
    x_list, y_list, z_list = [], [], []

    # Unpack angles, pos, orientation, rpy
    for _, pos, _, _ in full_trajectory:
        x_list.append(pos[0])
        y_list.append(pos[1])
        z_list.append(pos[2])

    N = len(full_trajectory)
    time = np.linspace(0, 3, N)

    plt.figure(figsize=(10,6))
    plt.plot(time, x_list, label="x")
    plt.plot(time, y_list, label="y")
    plt.plot(time, z_list, label="z")
    plt.xlabel("Time")
    plt.ylabel("Coordinates")
    plt.title("End-effector Velocity profiles")
    plt.legend()
    plt.grid(True)
    plt.show()

    print("\n" + "="*100)
    print(f"{'Step':<6} | {'T1':<7} {'T2':<7} {'T3':<7} {'T4':<7} {'T5':<7} {'T6':<7} | {'X':<6} {'Y':<6} {'Z':<6} | {'R':<6} {'P':<6} {'Y':<6}")
    print("="*100)
    
    for i, (angles, pos, _, rpy) in enumerate(full_trajectory):
        j1, j2, j3, j4, j5, j6 = np.degrees(angles)
        px, py, pz = pos
        r, p, y = rpy
        
        # Print every step (1-150)
        if i % 1 == 0 or i == len(full_trajectory) - 1:
            print(f"{i+1:<6} | {j1:<7.2f} {j2:<7.2f} {j3:<7.2f} {j4:<7.2f} {j5:<7.2f} {j6:<7.2f} | {px:<6.1f} {py:<6.1f} {pz:<6.1f} | {r:<6.1f} {p:<6.1f} {y:<6.1f}")

    visualize_trajectory(full_trajectory)
