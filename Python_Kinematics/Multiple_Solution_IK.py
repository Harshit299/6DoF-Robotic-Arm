import numpy as np
import math

# initial angles
T1 = 0
T2 = 0
T3 = 0
T4 = 0
T5 = 0
T6 = 0

# Convert input angles to radians
T1 = np.deg2rad(T1)
T2 = np.deg2rad(T2)
T3 = np.deg2rad(T3)
T4 = np.deg2rad(T4)
T5 = np.deg2rad(T5)
T6 = np.deg2rad(T6)

# Link lengths
a1, a2, a3 = 7.5, 30.0, 7.5

# Link offsets
d1, d2, d3 = 33.0, 32.0, 24.0

# DH Parameter Table
PT = [[T1,                  (90/180)*np.pi,   a1, d1], 
      [T2 + (90/180)*np.pi, 0,                a2, 0],
      [T3,                  (90/180)*np.pi,   a3, 0],
      [T4,                  -(90/180)*np.pi,  0,  d2],
      [T5,                  (90/180)*np.pi,   0,  0],
      [T6,                  0,                0,  d3]]

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

R0_3 = H0_3[0:3, 0:3]
R0_6 = H0_6[0:3, 0:3]

''' 
defining this function because T1, T2, T3 would be changed 
when wrist center reaches its correct positions, so new R0_3 would be needed'''
def new_R0_3(theta1, theta2, theta3):
    PT_local = [[theta1,                  (90/180)*np.pi,   a1, d1], 
                [theta2 + (90/180)*np.pi,        0,         a2,  0],
                [theta3,                  (90/180)*np.pi,   a3,  0]]
    
    H0_1_new = np.matrix([[np.cos(PT_local[0][0]), -np.sin(PT_local[0][0])*np.cos(PT_local[0][1]),  np.sin(PT_local[0][0])*np.sin(PT_local[0][1]), (PT_local[0][2])*np.cos(PT_local[0][0])],
                          [np.sin(PT_local[0][0]),  np.cos(PT_local[0][0])*np.cos(PT_local[0][1]), -np.cos(PT_local[0][0])*np.sin(PT_local[0][1]), (PT_local[0][2])*np.sin(PT_local[0][0])],
                          [0,                       np.sin(PT_local[0][1]),                         np.cos(PT_local[0][1]),                        PT_local[0][3]],
                          [0,                       0,                                              0,                                             1]])

    H1_2_new = np.matrix([[np.cos(PT_local[1][0]), -np.sin(PT_local[1][0])*np.cos(PT_local[1][1]),  np.sin(PT_local[1][0])*np.sin(PT_local[1][1]), (PT_local[1][2])*np.cos(PT_local[1][0])],
                          [np.sin(PT_local[1][0]),  np.cos(PT_local[1][0])*np.cos(PT_local[1][1]), -np.cos(PT_local[1][0])*np.sin(PT_local[1][1]), (PT_local[1][2])*np.sin(PT_local[1][0])],
                          [0,                       np.sin(PT_local[1][1]),                         np.cos(PT_local[1][1]),                        PT_local[1][3]],
                          [0,                       0,                                              0,                                             1]])

    H2_3_new = np.matrix([[np.cos(PT_local[2][0]), -np.sin(PT_local[2][0])*np.cos(PT_local[2][1]),  np.sin(PT_local[2][0])*np.sin(PT_local[2][1]), (PT_local[2][2])*np.cos(PT_local[2][0])],
                          [np.sin(PT_local[2][0]),  np.cos(PT_local[2][0])*np.cos(PT_local[2][1]), -np.cos(PT_local[2][0])*np.sin(PT_local[2][1]), (PT_local[2][2])*np.sin(PT_local[2][0])],
                          [0,                       np.sin(PT_local[2][1]),                         np.cos(PT_local[2][1]),                        PT_local[2][3]],
                          [0,                       0,                                              0,                                             1]])
    
    H0_3_new = H0_1_new @ H1_2_new @ H2_3_new

    return H0_3_new[0:3, 0:3]

# function to return target position and orientation
def return_targets():

    target_pos = np.array([-10.98, -29.39, 78.03]) # x, y, z wrt base frame

    roll = np.deg2rad(42)
    pitch = np.deg2rad(-18)
    yaw = np.deg2rad(64)

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

    # 1. EXTRINSIC rotation
    target_extrinsic = (Rz @ Ry @ Rx) @ R_zero

    # 2. INTRINSIC rotation
    Rx_EE = np.array([[1,      0,            0], 
                      [0, np.cos(yaw), -np.sin(yaw)], 
                      [0, np.sin(yaw), np.cos(yaw)]])
    
    Ry_EE = np.array([[np.cos(-pitch), 0, np.sin(-pitch)], 
                      [0,              1,       0], 
                      [-np.sin(-pitch),0, np.cos(-pitch)]])
    
    Rz_EE = np.array([[np.cos(roll), -np.sin(roll), 0], 
                      [np.sin(roll),  np.cos(roll), 0], 
                      [0,                  0,       1]])

    target_intrinsic = R_zero @ (Rx_EE @ Ry_EE @ Rz_EE)

    return target_pos, target_extrinsic, target_intrinsic

# IK function
def inverse_kinematics(target_pos, target_orientation):

    tool_offset = np.matrix([[0], [0], [d3]])
    wrist_pos = np.array(target_pos).reshape(3,1) - target_orientation @ tool_offset

    wrist_x_coord = float(wrist_pos[0, 0]) 
    wrist_y_coord = float(wrist_pos[1, 0]) 
    wrist_z_coord = float(wrist_pos[2, 0]) 

    print(f"\nWrist Center Coordinates: [{wrist_x_coord:.2f}, {wrist_y_coord:.2f}, {wrist_z_coord:.2f}]")
    
    valid_solutions = []

    # Calculate base horizontal radius to the target
    r1 = math.sqrt(wrist_x_coord**2 + wrist_y_coord**2)
    
    # 1. Shoulder configurations (Front / Back)
    theta1_front = math.atan2(wrist_y_coord, wrist_x_coord)
    theta1_back = theta1_front + np.pi

    shoulder_configs = [
        (theta1_front, r1 - a1, "Shoulder Left"),
        (theta1_back, -r1 - a1, "Shoulder Right") 
    ]

    # for loop for both shoulder configs
    for theta1, BC, shoulder_name in shoulder_configs:
        try:
            KI = wrist_z_coord - d1
            RK = math.sqrt(a3**2 + d2**2)
            QK = math.sqrt(KI**2 + BC**2)

            # Check if this configuration is physically reachable or not
            # If QK > arm span, reaching backwards is impossible
            if QK > (a2 + RK) or QK < abs(a2 - RK):
                continue

            beta = math.atan2(d2, a3)
            omega = math.atan2(KI, BC)

            gamma_base = math.acos(np.clip((RK**2 + a2**2 - QK**2)/(2*RK*a2), -1.0, 1.0))
            phi1_base = math.acos(np.clip((a2**2 + QK**2 - RK**2)/(2*a2*QK), -1.0, 1.0))

            # 2. Elbow configurations (Down / Up)
            elbow_configs = [
                (gamma_base, phi1_base, "Elbow Down"),
                (-gamma_base, -phi1_base, "Elbow Up")
            ]

            # for loop for both elbow configs
            for gamma, phi1, elbow_name in elbow_configs:
                theta3 = beta - np.pi + gamma
                theta2 = phi1 - np.pi/2 + omega

                # Get new R0_3 for this specific state
                R0_3_new = new_R0_3(theta1, theta2, theta3)
                R3_6_new = R0_3_new.T @ target_orientation

                r33 = np.clip(R3_6_new[2,2], -1.0, 1.0)
                theta5_base = math.acos(r33)

                # 3. Wrist configurations (Normal / rotated)
                wrist_configs = [
                    (theta5_base, 1, "wrist normal"),
                    (-theta5_base, -1, "wrist rotated")
                ]

                # for loop for both wrist configs
                for theta5, sign, wrist_name in wrist_configs:
                    
                    y_t4 = sign * R3_6_new[1,2]
                    x_t4 = sign * R3_6_new[0,2]
                    theta4 = math.atan2(y_t4, x_t4)

                    y_t6 = sign * R3_6_new[2,1]
                    x_t6 = sign * -R3_6_new[2,0]
                    theta6 = math.atan2(y_t6, x_t6)

                    valid_solutions.append({
                        "name": f"{shoulder_name}, {elbow_name}, {wrist_name}",
                        "angles": [np.rad2deg(t) for t in [theta1, theta2, theta3, theta4, theta5, theta6]]
                    })

        except ValueError:
            pass # Skipping this specific iteration if math fails somewhere

    return valid_solutions


target_pos, target_orientation, _ = return_targets()
solutions = inverse_kinematics(target_pos, target_orientation=target_orientation)

print(f"\nAll {len(solutions)} possible joint angles")
for i, sol in enumerate(solutions):
    angles = sol['angles']
    print(f"\nSolution {i+1}: {sol['name']}")
    print(f"J1: {angles[0]:8.2f}° | J2: {angles[1]:8.2f}° | J3: {angles[2]:8.2f}° | J4: {angles[3]:8.2f}° | J5: {angles[4]:8.2f}° | J6: {angles[5]:8.2f}°")
