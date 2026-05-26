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

    # p6 (Joint 4): move 7.97 cm forward from corner 2 towards joint 5
    forearm_vector = p7 - p5
    forearm_unit_vector = forearm_vector / np.linalg.norm(forearm_vector)
    p6 = p5 + (forearm_unit_vector * 7.97)
    
    # p8 (Joint 6): move 8 cm forward from Joint 5 towards EE
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
                          [0,                     np.sin(PT_new[0][1]),                       np.cos(PT_new[0][1]),                        PT_new[0][3]],
                          [0,                     0,                                          0,                                             1]])

    H1_2_new = np.matrix([[np.cos(PT_new[1][0]), -np.sin(PT_new[1][0])*np.cos(PT_new[1][1]),  np.sin(PT_new[1][0])*np.sin(PT_new[1][1]), (PT_new[1][2])*np.cos(PT_new[1][0])],
                          [np.sin(PT_new[1][0]),  np.cos(PT_new[1][0])*np.cos(PT_new[1][1]), -np.cos(PT_new[1][0])*np.sin(PT_new[1][1]), (PT_new[1][2])*np.sin(PT_new[1][0])],
                          [0,                       np.sin(PT_new[1][1]),                     np.cos(PT_new[1][1]),                        PT_new[1][3]],
                          [0,                       0,                                        0,                                             1]])

    H2_3_new = np.matrix([[np.cos(PT_new[2][0]), -np.sin(PT_new[2][0])*np.cos(PT_new[2][1]),  np.sin(PT_new[2][0])*np.sin(PT_new[2][1]), (PT_new[2][2])*np.cos(PT_new[2][0])],
                          [np.sin(PT_new[2][0]),  np.cos(PT_new[2][0])*np.cos(PT_new[2][1]), -np.cos(PT_new[2][0])*np.sin(PT_new[2][1]), (PT_new[2][2])*np.sin(PT_new[2][0])],
                          [0,                     np.sin(PT_new[2][1]),                     np.cos(PT_new[2][1]),                        PT_new[2][3]],
                          [0,                     0,                                        0,                                             1]])
    
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
    x, y, z = target_pos[0], target_pos[1], target_pos[2]

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
def generate_trajectory(start_pos, end_pos, start_roll, end_roll,
                        start_pitch, end_pitch, start_yaw, end_yaw, steps):
    
    trajectory_data = []
    t_values = np.linspace(0, 1, steps)

    for t in t_values:
        '''
        Cubic interpolation for smooth trajectory
        s = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        v = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        a = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
        BCs: s(0)=0, s(1)=1, v(0)=0=v(1), a(0)=0=a(1)
        a0 = 0, a1 = 0, a2 = 0, a3 = 10, a4 = -15, a5 = 6 '''

        # linear interpolation in space but nonlinear interpolation in time
        s = 10*(t**3) - 15*(t**4) + 6*(t**5) 

        cur_x = start_pos[0] + s * (end_pos[0] - start_pos[0])
        cur_y = start_pos[1] + s * (end_pos[1] - start_pos[1])
        cur_z = start_pos[2] + s * (end_pos[2] - start_pos[2])
        current_pos = [cur_x, cur_y, cur_z]

        v = 30*t**2 -60*t**3 + 30*t**4 # ds_dt

        vx = v * (end_pos[0] - start_pos[0])
        vy = v * (end_pos[1] - start_pos[1])
        vz = v * (end_pos[2] - start_pos[2])
        current_vel = [vx, vy, vz]

        a = 60*t - 180*t**2 + 120*t**3 # dv_dt
        
        ax = a * (end_pos[0] - start_pos[0])
        ay = a * (end_pos[1] - start_pos[1])
        az = a * (end_pos[2] - start_pos[2])
        current_acc = [ax, ay, az]

        cur_roll = start_roll + s * (end_roll - start_roll)
        cur_pitch = start_pitch + s * (end_pitch - start_pitch)
        cur_yaw = start_yaw + s * (end_yaw - start_yaw)
        cur_orientation = return_cur_orientation(cur_roll, cur_pitch, cur_yaw)
               
        cur_angles = inverse_kinematics(current_pos, cur_orientation)

        if cur_angles[0] is not None:
            trajectory_data.append((cur_angles, current_pos, current_vel, current_acc, cur_orientation, (cur_roll, cur_pitch, cur_yaw)))
        else:
            print(f"Skipping unreachable point at t={t:.3f}")

    return trajectory_data

# Function to visualize the trajectory using Matplotlib
def visualize_trajectory(trajectory_list, waypoints_list=None):
    plt.ion() 
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    path_x, path_y, path_z = [], [], []
    limit = 125 # axes limit
    
    # Extract Waypoints
    wp_x, wp_y, wp_z = [], [], []
    if waypoints_list:
        for wp in waypoints_list:
            pos = wp['pos']
            wp_x.append(pos[0])
            wp_y.append(pos[1])
            wp_z.append(pos[2])

    for angles, _, _, _,_, target_rpy in trajectory_list:
        ax.cla()

        th1, th2, th3, th4, th5, th6 = angles
        p1, p2, p3, p4, p5, p6, p7, p8, p9, R0_6 = extract_coords(th1, th2, th3, th4, th5, th6)
        
        # Update path using end effector position
        path_x.append(p9[0])
        path_y.append(p9[1])
        path_z.append(p9[2])
        
        # Plot trace & waypoints
        ax.plot(path_x, path_y, path_z, 'r--', linewidth=1, label='Path', zorder=1)
        if waypoints_list:
            ax.scatter(wp_x, wp_y, wp_z, color='green', s=100, marker='o', 
                      label='Waypoints', edgecolors='black', linewidths=2, zorder=2)
        
        # Extract individual arrays for plotting the arm
        line_pts = [p1, p2, p3, p4, p5, p6, p7, p8, p9]
        xs = [p[0] for p in line_pts]
        ys = [p[1] for p in line_pts]
        zs = [p[2] for p in line_pts]
        
        # Plot the arm connecting everything
        ax.plot(xs, ys, zs, '-', color='dimgrey', linewidth=4, zorder=3)
        
        # Plot the specific colored dots for each joint and corner
        colors = ['blue', 'black', 'red', 'green', 'black', 'orange', 'cyan', 'magenta', 'purple']
        for pt, col in zip(line_pts, colors):
            ax.scatter(*pt, color=col, s=80, zorder=4, depthshade=False)
            
        # Draw the End Effector Orientation Arrows (Red=X, Green=Y, Blue=Z)
        R = np.asarray(R0_6)
        arrow_len = 35
        ax.quiver(p9[0], p9[1], p9[2], R[0,0], R[1,0], R[2,0], color='r', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        ax.quiver(p9[0], p9[1], p9[2], R[0,1], R[1,1], R[2,1], color='g', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        ax.quiver(p9[0], p9[1], p9[2], R[0,2], R[1,2], R[2,2], color='b', length=arrow_len, normalize=True, linewidth=2, zorder=5)
        
        # Base reference grid lines
        ax.plot([-limit, limit], [0, 0], [0, 0], 'k--', alpha=0.3)
        ax.plot([0, 0], [-limit, limit], [0, 0], 'k--', alpha=0.3)
        
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

    # Define waypoints
    pt_home  = {'pos': [-35, 40, 55.0], 'roll': 52,  'pitch': -12, 'yaw':  25}
    pt_pick  = {'pos': [40, -40, 50],   'roll': -10, 'pitch':  14, 'yaw':  -24} 
    pt_lift  = {'pos': [25, 50, 30],    'roll': 41,  'pitch': -37, 'yaw': -45}
    pt_place = {'pos': [50, -50, 10],   'roll': -80, 'pitch':  18, 'yaw': -64}    
    
    traj1 = generate_trajectory(pt_home['pos'], pt_pick['pos'], 
                                pt_home['roll'], pt_pick['roll'], 
                                pt_home['pitch'], pt_pick['pitch'], 
                                pt_home['yaw'], pt_pick['yaw'], 
                                steps=50)
        
    traj2 = generate_trajectory(pt_pick['pos'], pt_lift['pos'], 
                                pt_pick['roll'], pt_lift['roll'], 
                                pt_pick['pitch'], pt_lift['pitch'], 
                                pt_pick['yaw'], pt_lift['yaw'], 
                                steps=50)
        
    traj3 = generate_trajectory(pt_lift['pos'], pt_place['pos'], 
                                pt_lift['roll'], pt_place['roll'], 
                                pt_lift['pitch'], pt_place['pitch'], 
                                pt_lift['yaw'], pt_place['yaw'], 
                                steps=50)
    
    full_trajectory = traj1 + traj2 + traj3

    # Empty lists to generate profiles
    x_list, y_list, z_list = [], [], []
    vx_list, vy_list, vz_list = [], [], []
    ax_list, ay_list, az_list = [], [], []

    # Unpack pos, vel, acc for each point in trajectory
    for _, pos, vel, acc, _, _ in full_trajectory:
        x_list.append(pos[0])
        y_list.append(pos[1])
        z_list.append(pos[2])

        vx_list.append(vel[0])
        vy_list.append(vel[1])
        vz_list.append(vel[2])

        ax_list.append(acc[0])
        ay_list.append(acc[1])
        az_list.append(acc[2])

    N = len(full_trajectory)
    time = np.linspace(0, 3, N)

    # Velocity profiles
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

    # Acceleration profiles
    plt.figure(figsize=(10,6))
    plt.plot(time, vx_list, label="x")
    plt.plot(time, vy_list, label="y")
    plt.plot(time, vz_list, label="z")
    plt.xlabel("Time")
    plt.ylabel("Velocity")
    plt.title("End-effector Acceleration profiles")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Jerk profiles
    plt.figure(figsize=(10,6))
    plt.plot(time, ax_list, label="x")
    plt.plot(time, ay_list, label="y")
    plt.plot(time, az_list, label="z")
    plt.xlabel("Time")
    plt.ylabel("Acceleration")
    plt.title("End-effector Jerk profiles")
    plt.legend()
    plt.grid(True)
    plt.show()

    print("\n" + "="*100)
    print(f"{'Step':<6} | {'T1':<7} {'T2':<7} {'T3':<7} {'T4':<7} {'T5':<7} {'T6':<7} | {'X':<6} {'Y':<6} {'Z':<6} | {'R':<6} {'P':<6} {'Y':<6}")
    print("="*100)
    
    for i, (angles, pos, _, _, _, rpy) in enumerate(full_trajectory):
        j1, j2, j3, j4, j5, j6 = np.degrees(angles)
        px, py, pz = pos
        r, p, y = rpy
        
        # Print every step
        if i % 1 == 0 or i == len(full_trajectory) - 1:
            print(f"{i+1:<6} | {j1:<7.2f} {j2:<7.2f} {j3:<7.2f} {j4:<7.2f} {j5:<7.2f} {j6:<7.2f} | {px:<6.1f} {py:<6.1f} {pz:<6.1f} | {r:<6.1f} {p:<6.1f} {y:<6.1f}")

    visualize_trajectory(full_trajectory, waypoints_list=[pt_home, pt_pick])
