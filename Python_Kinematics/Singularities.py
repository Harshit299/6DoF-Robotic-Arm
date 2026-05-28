import numpy as np
import matplotlib.pyplot as plt

# Link offsets
a1, a2, a3 = 7.5, 30.0, 7.5

# Link lengths
d1, d2, d3 = 33.0, 32.0, 24.0

def compute_jacobian(T1, T2, T3, T4, T5, T6):

    T1 = np.deg2rad(T1)
    T2 = np.deg2rad(T2)
    T3 = np.deg2rad(T3)
    T4 = np.deg2rad(T4)
    T5 = np.deg2rad(T5)
    T6 = np.deg2rad(T6)

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

    # Calculate Jacobians

    # for joint 1
    d0_6 = np.array(H0_6[0:3, 3]).flatten()
    d0_0 = np.array([0,0,0])
    J_v1 = np.cross(np.array([0,0,1]), (d0_6 - d0_0))
    J_w1 = np.array([0,0,1])

    # for joint 2
    R0_1 = np.array(H0_1[0:3, 0:3])
    d0_1 = np.array(H0_1[0:3, 3]).flatten()
    J_v2 = np.cross(R0_1[0:3, 2], (d0_6 - d0_1))
    J_w2 = np.array(R0_1[0:3, 2])

    # for joint 3
    R0_2 = np.array(H0_2[0:3, 0:3])
    d0_2 = np.array(H0_2[0:3, 3]).flatten()
    J_v3 = np.cross(R0_2[0:3, 2], (d0_6 - d0_2))
    J_w3 = np.array(R0_2[0:3, 2])

    # for joint 4
    R0_3 = np.array(H0_3[0:3, 0:3])
    d0_3 = np.array(H0_3[0:3, 3]).flatten()
    J_v4 = np.cross(R0_3[0:3, 2], (d0_6 - d0_3))
    J_w4 = np.array(R0_3[0:3, 2])

    # for joint 5
    R0_4 = np.array(H0_4[0:3, 0:3])
    d0_4 = np.array(H0_4[0:3, 3]).flatten()
    J_v5 = np.cross(R0_4[0:3, 2], (d0_6 - d0_4))
    J_w5 = np.array(R0_4[0:3, 2])

    # for joint 6
    R0_5 = np.array(H0_5[0:3, 0:3])
    d0_5 = np.array(H0_5[0:3, 3]).flatten()
    J_v6 = np.cross(R0_5[0:3, 2], (d0_6 - d0_5))
    J_w6 = np.array(R0_5[0:3, 2])

    Jv = np.column_stack([J_v1, J_v2, J_v3, J_v4, J_v5, J_v6])
    Jw = np.column_stack([J_w1, J_w2, J_w3, J_w4, J_w5, J_w6])
    J = np.vstack([Jv, Jw])

    return J, d0_6


# PLOTTING LOGIC ============================================

fig = plt.figure(figsize=(16, 7))

# PLOT 1: ARM SINGULARITIES IN CARTESIAN SPACE 
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title("Arm Singularities", fontsize=14, fontweight='bold')
ax1.set_xlabel("Global X (cm)")
ax1.set_ylabel("Global Y (cm)")
ax1.set_zlabel("Global Z (cm)")

# lists to hold singular points
x_sing, y_sing, z_sing = [], [], []

# Calculating Arm singularities
# Step sizes of 10 degrees
for t1 in range(-180, 181, 10):  # t1 doesn't affect singlarities, but we loop through it for completeness
    for t2 in range(-90, 91, 10):
        for t3 in range(-90, 91, 10):
            J, ee_pos = compute_jacobian(t1, t2, t3, 0, 0, 0)
            
            # Extracting only the arm part of the Jacobian (top-left 3x3)
            J_arm = J[0:3, 0:3]
            cond_arm = np.linalg.cond(J_arm) # condition number
            
            # High condition number means danger (Singularity)
            if cond_arm > 50: # we can adjust this threshold according to specific application
                x_sing.append(ee_pos[0])
                y_sing.append(ee_pos[1])
                z_sing.append(ee_pos[2])

# Plot the singular points as a red points
ax1.scatter(x_sing, y_sing, z_sing, c='red', s=5, alpha=0.5, label="Singular Configurations")
ax1.legend()


# PLOT 2: WRIST SINGULARITIES IN JOINT SPACE
# Theta 4 and Theta 6 don't affect wrist singularities

ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title("Wrist Singularity vs Joint Angles", fontsize=14, fontweight='bold')
ax2.set_xlabel("Theta 4 (deg)")
ax2.set_ylabel("Theta 5 (deg)")
ax2.set_zlabel("Condition Number (Danger Level)")

t4_vals = np.arange(-180, 181, 3)
t5_vals = np.arange(-180, 181, 3)
T4, T5 = np.meshgrid(t4_vals, t5_vals)
Cond_Grid = np.zeros_like(T4)

# Calculating Wrist singularities
for i in range(len(t4_vals)):
    for j in range(len(t5_vals)):
        # any values of t1, t2, t3 will work
        J, _ = compute_jacobian(0, 30, -30, T4[i,j], T5[i,j], 0) 
        
        # Extract only wrist part of jacobian (bottom-right 3x3)
        J_wrist = J[3:6, 3:6]
        
        # Calculate condition number and limit it to 300
        try:
            c = np.linalg.cond(J_wrist)
            Cond_Grid[i,j] = min(c, 300) 
        except np.linalg.LinAlgError:
            Cond_Grid[i,j] = 300

# Plot the 3D surface
surf = ax2.plot_surface(T4, T5, Cond_Grid, cmap='magma', edgecolor='none')
fig.colorbar(surf, ax=ax2, shrink=0.5, aspect=10, label='Danger level')
plt.tight_layout()
plt.show()
