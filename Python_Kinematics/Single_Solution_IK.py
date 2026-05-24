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

# Link offsets
a1, a2, a3 = 7.5, 30.0, 7.5

# Link lengths
d1, d2, d3 = 30.0, 32.0, 24.0

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

    # 1. EXTRINSIC rotation: first rotate about global x --> then about global y --> global z
    target_extrinsic = (Rz @ Ry @ Rx) @ R_zero

    # 2. INTRINSIC rotation: first rotate about local z --> then about new local y --> newest local x
    Rx_EE = np.array([[1, 0, 0], [0, np.cos(yaw), -np.sin(yaw)], [0, np.sin(yaw), np.cos(yaw)]])
    Ry_EE = np.array([[np.cos(-pitch), 0, np.sin(-pitch)], [0, 1, 0], [-np.sin(-pitch), 0, np.cos(-pitch)]])
    Rz_EE = np.array([[np.cos(roll), -np.sin(roll), 0], [np.sin(roll), np.cos(roll), 0], [0, 0, 1]])

    target_intrinsic = R_zero @ (Rx_EE @ Ry_EE @ Rz_EE)

    return target_pos, target_extrinsic, target_intrinsic

def inverse_kinematics(target_pos, target_orientation):

    tool_offset = np.matrix([[0], [0], [d3]])
    wrist_pos = np.array(target_pos).reshape(3,1) - target_orientation @ tool_offset

    wrist_x_coord = float(wrist_pos[0, 0]) # wrt base frame
    wrist_y_coord = float(wrist_pos[1, 0]) # wrt base frame
    wrist_z_coord = float(wrist_pos[2, 0]) # wrt base frame

    r1 = math.sqrt(wrist_x_coord**2 + wrist_y_coord**2)
    theta1 = math.atan2(wrist_y_coord, wrist_x_coord)

    print("\nWrist Center Coordinates (wrt base frame):")
    print(wrist_x_coord, wrist_y_coord, wrist_z_coord)

    
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

        print("\nR3_6:", R3_6_new)
        print("\ntarget orientation: ", target_orientation)

        theta4 = math.atan2(R3_6_new[1,2], R3_6_new[0,2])
        theta5 = math.acos(R3_6_new[2,2])
        theta6 = math.atan2(R3_6_new[2,1], -R3_6_new[2,0])

        return theta1, theta2, theta3, theta4, theta5, theta6

    except ValueError:
        print("Target position is out of reach for the manipulator.")
        return None, None, None, None, None, None

target_pos, target_orientation, _ = return_targets()
theta1, theta2, theta3, theta4, theta5, theta6 = inverse_kinematics(target_pos=target_pos, target_orientation=target_orientation)
if theta1 is not None:
    print("\nCalculated Joint Angles (Degrees):")
    print(np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4), np.rad2deg(theta5), np.rad2deg(theta6))