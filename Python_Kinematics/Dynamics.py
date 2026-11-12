import numpy as np
import math
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

'''
Units:
  mass    - kg
  length  - m
  inertia - kg * m^2
  angles  - rad
'''

# =====================================================================
# 1. DYNAMIC PARAMETERS (LINKS 1 TO 6)
# =====================================================================
m_base  = 13.634
m_link1 = 12.259
m_link2 = 16.077
m_link3 = 7.320
m_link4 = 5.820
m_link5 = 1.2072
m_link6 = 0.074288

# Base link is stationary, only moving links 1 to 6 are included
m = [m_link1, m_link2, m_link3, m_link4, m_link5, m_link6]

com_base  = np.array([-24.587e-3,  -0.113e-3,  -77.833e-3])
com_link1 = np.array([-52.524e-3, -85.217e-3,    0.706e-3])
com_link2 = np.array([-190.017e-3,  0.480e-3,  -10.653e-3])
com_link3 = np.array([-37.160e-3,   1.173e-3,  -81.675e-3])
com_link4 = np.array([ -2.487e-3, 128.740e-3,    5.982e-3])
com_link5 = np.array([  1.161e-3,   1.553e-3,  -58.703e-3])
com_link6 = np.array([  2.802e-3,  -0.227e-3, -136.898e-3])

com = [com_link1, com_link2, com_link3, com_link4, com_link5, com_link6]

I_base = np.array([[ 0.053406,   -0.00026284,  0.0025009 ],
                   [-0.00026284,  0.076734,    9.8013e-05],
                   [ 0.0025009,   9.8013e-05,  0.080661  ]])

I_link1 = np.array([[0.11869,   0.018675,  0.0012185],
                    [0.018675,  0.10528,   0.0021687],
                    [0.0012185, 0.0021687, 0.073332 ]])

I_link2 = np.array([[ 0.079946,   -1.0509e-05, -0.01131   ],
                    [-1.0509e-05,  0.22994,    -0.00016923],
                    [-0.01131,    -0.00016923,  0.18226   ]])

I_link3 = np.array([[ 0.022615,   -0.00010223,  0.0046415 ],
                    [-0.00010223,  0.027884,   -1.1208e-05],
                    [ 0.0046415,  -1.1208e-05,  0.023179  ]])

I_link4 = np.array([[ 0.032618,   -0.00040859,  2.1488e-05],
                    [-0.00040859,  0.010466,   -0.00033652],
                    [ 2.1488e-05, -0.00033652,  0.030701  ]])

I_link5 = np.array([[ 0.0014824,  6.8815e-07, -1.298e-05 ],
                    [ 6.8815e-07, 0.0016274,  -3.4122e-05],
                    [-1.298e-05, -3.4122e-05,  0.00083285]])

I_link6 = np.array([[ 0.00014064, -3.6954e-13, -2.6419e-06],
                    [-3.6954e-13,  0.00014069,  1.85e-11  ],
                    [-2.6419e-06,  1.85e-11,    8.4273e-06]])

I = [I_link1, I_link2, I_link3, I_link4, I_link5, I_link6]


# Standard DH link offsets and lengths (meters)
a1, a2, a3 = 0.075, 0.30, 0.075
d1, d2, d3 = 0.33, 0.32, 0.24

# =====================================================================
# 2. KINEMATICS (FORWARD & INVERSE KINEMATICS)
# =====================================================================
def forward_kinematics(t1, t2, t3, t4, t5, t6):
    # Takes 6 joint angles in radians and returns R_list and p_list

    PT = [[t1,                0.5 * np.pi, a1,  d1 ], # joint1
          [t2 + 0.5 * np.pi,  0.0,         a2,  0.0], # joint2
          [t3,                0.5 * np.pi, a3,  0.0], # joint3
          [t4,               -0.5 * np.pi, 0.0, d2 ], # joint4
          [t5,                0.5 * np.pi, 0.0, 0.0], # joint5
          [t6,                0.0,         0.0, d3 ]] # joint6

    R_list, p_list = [], []
    for i in range(6):
        theta, alpha, a, d = PT[i]
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)

        # for standard DH
        R_i = np.array([
            [ct, -st * ca,  st * sa],
            [st,  ct * ca, -ct * sa],
            [0.0,     sa,       ca ]
        ])

        # # for modified DH
        # R_i = np.array([
        #     [ct,      -st,      0.0],
        #     [st * ca,  ct * ca, -sa],
        #     [st * sa,  ct * sa,  ca]
        # ])

        p_i = np.array([a, -d * sa, d * ca])
        p_i = np.array([a * ct, a * st, d])
        R_list.append(R_i)
        p_list.append(p_i)

    return R_list, p_list


def get_end_effector_pose(q):
    # Computes end-effector Cartesian position (3,) and orientation matrix (3,3) from q (rad)
    R_list, p_list = forward_kinematics(*q)
    T = np.eye(4)
    for i in range(6):
        T_i = np.eye(4)
        T_i[:3, :3] = R_list[i]
        T_i[:3, 3]  = p_list[i]
        T = T @ T_i
    return T[:3, 3], T[:3, :3]


def new_R0_3(theta1, theta2, theta3):
    PT_new = [
        [theta1,               0.5 * np.pi, a1, d1 ],
        [theta2 + 0.5 * np.pi, 0.0,         a2, 0.0],
        [theta3,               0.5 * np.pi, a3, 0.0]
    ]
    H0_3 = np.eye(4)
    for th, al, a, d in PT_new:
        ct, st = np.cos(th), np.sin(th)
        ca, sa = np.cos(al), np.sin(al)
        H_i = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,     sa,       ca,  d     ],
            [0.0,    0.0,      0.0,  1.0   ]
        ])
        H0_3 = H0_3 @ H_i
    return H0_3[:3, :3]


def inverse_kinematics(target_pos, target_orientation):
    """
    Computes 6 joint angles (radians) for a 3D target_pos [x, y, z] (meters)
    and a 3x3 target_orientation rotation matrix.
    """
    tool_offset = np.array([0.0, 0.0, d3])
    wrist_pos = np.asarray(target_pos, dtype=float).reshape(3) - np.asarray(target_orientation, dtype=float) @ tool_offset

    wrist_x_coord, wrist_y_coord, wrist_z_coord = wrist_pos[0], wrist_pos[1], wrist_pos[2]

    r1 = math.sqrt(wrist_x_coord**2 + wrist_y_coord**2)
    theta1 = math.atan2(wrist_y_coord, wrist_x_coord)

    try:
        BC = r1 - a1
        KI = wrist_z_coord - d1
        RK = math.sqrt(a3**2 + d2**2)
        QK = math.sqrt(KI**2 + BC**2)

        # Check if target is within geometric reach
        cos_gamma = (RK**2 + a2**2 - QK**2) / (2.0 * RK * a2)
        cos_phi1  = (a2**2 + QK**2 - RK**2) / (2.0 * a2 * QK)
        if abs(cos_gamma) > 1.0 + 1e-6 or abs(cos_phi1) > 1.0 + 1e-6:
            raise ValueError("Target position is outside the manipulator workspace.")

        beta   = math.atan2(d2, a3)
        gamma  = math.acos(np.clip(cos_gamma, -1.0, 1.0))
        theta3 = beta - np.pi + gamma

        omega  = math.atan2(KI, BC)
        phi1   = math.acos(np.clip(cos_phi1, -1.0, 1.0))
        theta2 = phi1 - 0.5 * np.pi + omega

        R0_3_new = new_R0_3(theta1=theta1, theta2=theta2, theta3=theta3)
        R3_6_new = R0_3_new.T @ target_orientation

        theta5 = math.acos(np.clip(R3_6_new[2, 2], -1.0, 1.0))
        if abs(math.sin(theta5)) > 1e-6:
            theta4 = math.atan2(R3_6_new[1, 2], R3_6_new[0, 2])
            theta6 = math.atan2(R3_6_new[2, 1], -R3_6_new[2, 0])
        else:
            theta4 = 0.0
            theta6 = math.atan2(-R3_6_new[1, 0], R3_6_new[0, 0])

        return np.array([theta1, theta2, theta3, theta4, theta5, theta6])

    except ValueError as err:
        print(f"[IK Error]: {err}")
        return None


# =====================================================================
# 3. INVERSE DYNAMICS (RNEA) & FORWARD DYNAMICS (RK4)
# =====================================================================
def rnea(q, dq, ddq, g_vec=np.array([0.0, 0.0, 9.81]), f_tip=np.zeros(3), n_tip=np.zeros(3), m_load=0.0):
    """
    Inverse Dynamics via Recursive Newton-Euler Algorithm.
    m_load: Point-mass payload (kg) attached at the end-effector tip (Frame 6 origin).
    """
    z_cap = np.array([0.0, 0.0, 1.0])
    R, P = forward_kinematics(q[0], q[1], q[2], q[3], q[4], q[5])

    w_prev  = np.zeros(3)
    dw_prev = np.zeros(3)
    a_prev  = g_vec.copy()

    w   = [np.zeros(3) for _ in range(6)]
    dw  = [np.zeros(3) for _ in range(6)]
    a   = [np.zeros(3) for _ in range(6)]
    a_c = [np.zeros(3) for _ in range(6)]
    F   = [np.zeros(3) for _ in range(6)]
    N   = [np.zeros(3) for _ in range(6)]

    # Forward Pass (link1 to Tip)
    for i in range(6):
        w[i]   = R[i].T @ (w_prev + dq[i] * z_cap)
        dw[i]  = R[i].T @ (dw_prev + ddq[i] * z_cap + np.cross(w_prev, dq[i] * z_cap))
        p_star = R[i].T @ P[i]
        a[i]   = R[i].T @ a_prev + np.cross(dw[i], p_star) + np.cross(w[i], np.cross(w[i], p_star))
        a_c[i] = a[i] + np.cross(dw[i], com[i]) + np.cross(w[i], np.cross(w[i], com[i]))

        F[i] = m[i] * a_c[i]
        N[i] = I[i] @ dw[i] + np.cross(w[i], I[i] @ w[i])

        w_prev, dw_prev, a_prev = w[i], dw[i], a[i]

    # Backward Pass (Tip to link1)
    tau = np.zeros(6)
    # Include external tip force/moment AND attached payload mass (m_load * tip acceleration a[5])
    f_next = f_tip.copy() + m_load * a[5] # currently this is for link6
    n_next = n_tip.copy() # currently this is for link6

    for i in range(5, -1, -1):
        f_next = R[i+1] @ f_next if i < 5 else f_next # i = 5 is the last link, so no R[i+1] for it
        n_next = R[i+1] @ n_next if i < 5 else n_next
        p_star = R[i].T @ P[i]

        f_i = F[i] + f_next
        n_i = N[i] + n_next + np.cross(p_star + com[i], F[i]) + np.cross(p_star, f_next)

        tau[i] = np.dot(R[i] @ n_i, z_cap)
        f_next, n_next = f_i, n_i

    return tau


def compute_mass_matrix(q, m_load=0.0):
    # we accelerate one joint at a time is inside this function
    M = np.zeros((6, 6))
    for j in range(6):
        e_j = np.zeros(6)
        e_j[j] = 1.0
        M[:, j] = rnea(q, np.zeros(6), e_j, g_vec=np.zeros(3), m_load=m_load)
    return M


def forward_dynamics(q, dq, tau, g_vec=np.array([0.0, 0.0, 9.81]), f_tip=np.zeros(3), n_tip=np.zeros(3), m_load=0.0):
    # Solves M(q) * ddq = tau - b(q, dq) for joint accelerations ddq.
    b = rnea(q, dq, np.zeros(6), g_vec, f_tip, n_tip, m_load=m_load)
    M = compute_mass_matrix(q, m_load=m_load)
    ddq = np.linalg.solve(M, tau - b)
    return dq, ddq


def rk4_step(q, dq, tau, dt, g_vec=np.array([0.0, 0.0, 9.81]), f_tip=np.zeros(3), n_tip=np.zeros(3), m_load=0.0):
    """ 
    integrates state (q, dq) forward by dt using 4th-Order Runge-Kutta.
    inside this function all 6 joints move simultaneously 
    """

    k1_q, k1_dq = forward_dynamics(q, dq, tau, g_vec, f_tip, n_tip, m_load=m_load)
    k2_q, k2_dq = forward_dynamics(q + 0.5 * dt * k1_q, dq + 0.5 * dt * k1_dq, tau, g_vec, f_tip, n_tip, m_load=m_load)
    k3_q, k3_dq = forward_dynamics(q + 0.5 * dt * k2_q, dq + 0.5 * dt * k2_dq, tau, g_vec, f_tip, n_tip, m_load=m_load)
    k4_q, k4_dq = forward_dynamics(q + dt * k3_q, dq + dt * k3_dq, tau, g_vec, f_tip, n_tip, m_load=m_load)

    # integrating acceleration to get velocity at next time step
    dq_next = dq + (dt / 6.0) * (k1_dq + 2.0 * k2_dq + 2.0 * k3_dq + k4_dq)

    # integrating velocity to get position at next time step
    q_next  = q  + (dt / 6.0) * (k1_q  + 2.0 * k2_q  + 2.0 * k3_q  + k4_q)

    return q_next, dq_next


# =====================================================================
# 4. CARTESIAN IK TRAJECTORY GENERATOR
# =====================================================================
def generate_trajectory(start_pos, end_pos, target_rot, total_time, dt):
    """
    Interpolates a straight Cartesian line from start_pos to end_pos using a smooth
    quintic profile, solves Inverse Kinematics at every step, and differentiates
    to get desired joint positions (q), velocities (dq), and accelerations (ddq).
    """
    steps = int(total_time / dt) # 5.0/0.005 = 1000 steps
    start_pos = np.asarray(start_pos, dtype=float)
    end_pos   = np.asarray(end_pos, dtype=float)

    q_traj = np.zeros((steps, 6)) # 1000x6 matrix to hold joint angles at EACH time step
    for step in range(steps):
        t = step * dt
        norm_t = np.clip(t / total_time, 0.0, 1.0)

        # Quintic time scaling: smooth 0 -> 1 with zero velocity/acceleration at ends
        s = 10.0 * norm_t**3 - 15.0 * norm_t**4 + 6.0 * norm_t**5
        pos_t = start_pos + s * (end_pos - start_pos) # position at time t along the straight line

        q_sol = inverse_kinematics(pos_t, target_rot)

        if q_sol is None:
            raise RuntimeError(f"IK failed at step {step}, pos={pos_t}")
        
        q_traj[step] = q_sol # store joint angles for this time step (picking one row of q_traj matrix)

    # Unwrap angles to avoid +-pi jumps before taking numerical derivatives
    q_traj   = np.unwrap(q_traj, axis=0)
    dq_traj  = np.gradient(q_traj, dt, axis=0)
    ddq_traj = np.gradient(dq_traj, dt, axis=0)
    return q_traj, dq_traj, ddq_traj


# =====================================================================
# 5. COPPELIASIM DYNAMIC SIMULATION
# =====================================================================
if __name__ == "__main__":
    # Verify Mass Matrix Symmetry & Eigenvalues
    q_test = np.array([0.3, -0.5, 0.8, -0.2, 0.6, -0.7])
    M_test = compute_mass_matrix(q_test)
    symmetry_error = np.max(np.abs(M_test - M_test.T))
    eigenvalues = np.linalg.eigvals(M_test)

    print(f"Max Symmetry Error |M - M.T|: {symmetry_error:.2e}")
    print(f"Eigenvalues of M(q): {np.round(eigenvalues, 6)}")

    # Simulation & Trajectory Setup
    dt = 0.005          # 5 ms time step
    total_time = 5.0    # 5 seconds duration
    steps = int(total_time / dt)
    g_vec = np.array([0.0, 0.0, 9.81])
    payload_mass = 1000.0

    # Define reachable Cartesian start & end positions (meters) and tool orientation
    start_pos = [0.45, -0.45, 0.25]
    end_pos   = [0.45,  0.30, 0.60]
    _, target_rot = get_end_effector_pose(np.deg2rad([20.0, 10.0, 35.0, 0.0, 0.0, 0.0]))

    # Pre-compute the desired trajectory (q_d, dq_d, ddq_d) via IK
    q_traj, dq_traj, ddq_traj = generate_trajectory(start_pos, end_pos, target_rot, total_time, dt)

    def get_desired_trajectory(step_idx):
        idx = min(step_idx, len(q_traj) - 1)
        return q_traj[idx], dq_traj[idx], ddq_traj[idx]

    # Computed Torque Control PD Gains (critically damped: Kd = 2 * sqrt(Kp))
    Kp = np.diag([100.0, 100.0, 100.0, 64.0, 64.0, 36.0])
    Kd = np.diag([ 20.0,  20.0,  20.0, 16.0, 16.0, 12.0])

    # Initialize actual arm state at the start of the trajectory
    q_actual, dq_actual, _ = get_desired_trajectory(0)

    # Connect to CoppeliaSim
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.setStepping(True)

    joint_names = ['/Joint1', '/Joint2', '/Joint3', '/Joint4', '/Joint5', '/Joint6']
    joint_handles = [sim.getObject(name) for name in joint_names]

    # Data loggers
    time_log = []
    tau_total_log, tau_grav_log, tau_cor_log, tau_iner_log = [], [], [], []
    q_err_log, power_log = [], []

    print("Starting CoppeliaSim dynamic simulation...")
    sim.startSimulation()

    for step in range(steps):
        t = step * dt

        # Get desired state (q_d, dq_d, ddq_d) from IK Trajectory Planner
        q_d, dq_d, ddq_d = get_desired_trajectory(step)

        # Computed Torque Control Law (Inverse Dynamics + PD Feedback)
        e  = q_d - q_actual
        de = dq_d - dq_actual
        ddq_cmd = ddq_d + Kp @ e + Kd @ de
        tau_cmd = rnea(q_actual, dq_actual, ddq_cmd, g_vec, m_load=payload_mass)

        # Decompose torques (Gravity vs Coriolis vs Inertia)
        tau_g = rnea(q_actual, np.zeros(6), np.zeros(6), g_vec, m_load=payload_mass)
        tau_c = rnea(q_actual, dq_actual, np.zeros(6), np.zeros(3), m_load=payload_mass)
        tau_i = rnea(q_actual, np.zeros(6), ddq_cmd, np.zeros(3), m_load=payload_mass)

        # Forward Dynamics + RK4 Integration (Physical arm response)
        q_actual, dq_actual = rk4_step(q_actual, dq_actual, tau_cmd, dt, g_vec, m_load=payload_mass)

        # Update CoppeliaSim 3D Visualization
        for i in range(6):
            sim.setJointPosition(joint_handles[i], float(q_actual[i]))
        sim.step()

        # Log data
        time_log.append(t)
        tau_total_log.append(tau_cmd)
        tau_grav_log.append(tau_g)
        tau_cor_log.append(tau_c)
        tau_iner_log.append(tau_i)
        q_err_log.append(e)
        power_log.append(tau_cmd * dq_actual)

    sim.stopSimulation()
    print("Simulation complete!")

    # Plot Joint Torques and Torque Decomposition
    time_arr = np.array(time_log)
    tau_total_arr = np.array(tau_total_log)
    tau_grav_arr  = np.array(tau_grav_log)
    tau_cor_arr   = np.array(tau_cor_log)
    tau_iner_arr  = np.array(tau_iner_log)

    fig, ax = plt.subplots(figsize=(12, 5))

    for i in range(6):
        ax.plot(time_arr, tau_total_arr[:, i], label=f'Joint {i+1}')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Total Torque (N·m)')
    ax.set_title('6-DOF Joint Torques Along Trajectory')
    ax.grid(True)
    ax.legend(loc='upper right', ncol=3)

    plt.tight_layout()
    plt.show()