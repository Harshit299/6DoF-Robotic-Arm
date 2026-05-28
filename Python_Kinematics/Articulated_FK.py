import numpy as np

# Link lengths
a1, a2, a3 = 7.5, 30.0, 7.5

# Link offsets
d1, d2, d3 = 33.0, 32.0, 24.0

def forward_kinematics(t1_deg, t2_deg, t3_deg, t4_deg, t5_deg, t6_deg):

    # Convert input angles to radians
    T1 = np.deg2rad(t1_deg)
    T2 = np.deg2rad(t2_deg)
    T3 = np.deg2rad(t3_deg)
    T4 = np.deg2rad(t4_deg)
    T5 = np.deg2rad(t5_deg)
    T6 = np.deg2rad(t6_deg)

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

    R0_6 = H0_6[0:3, 0:3]

    # Extract Positions
    tool_offset = np.matrix([[0], [0], [d3]])
    wrist_pos = H0_6[0:3, 3] - R0_6 @ tool_offset
    end_effector_pos = H0_6[0:3, 3]

    return R0_6, end_effector_pos, wrist_pos


# rot_matrix, end_pos, wrist_pos = forward_kinematics(0,0,0,0,0,0)
rot_matrix, end_pos, wrist_pos = forward_kinematics(48.61, 205.05, -103.85, 14.45, 64.48, -100.81)

print("Rotation Matrix:\n", rot_matrix)
print("\nEnd Effector Position [X, Y, Z]:\n", end_pos)
print("\nWrist Position [X, Y, Z]:\n", wrist_pos)
