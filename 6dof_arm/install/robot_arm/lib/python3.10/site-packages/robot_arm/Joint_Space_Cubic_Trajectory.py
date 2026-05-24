import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


# Link lengths
a1, a2, a3 = 7.5, 30.0, 7.5

# Link offsets
d1, d2, d3 = 33.0, 32.0, 24.0

class TrajectoryNode(Node):

    def __init__(self):
        super().__init__('joint_space_quintic_trajectory_node')

        self.angles_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_publisher = self.create_publisher(Marker, '/ee_path', 10)
        self.path_points = []
        self.timer = self.create_timer(0.05, self.timer_callback)   # 20 Hz

        #  Waypoints 
        self.pt_home  = {'pos': [-35, 40, 55.0], 'roll': 52,  'pitch': -12, 'yaw':  25}
        self.pt_pick  = {'pos': [40, -40, 50],   'roll': -10, 'pitch':  14, 'yaw':  -24} 
        self.pt_lift  = {'pos': [25, 50, 30],    'roll': 41,  'pitch': -37, 'yaw': -45}
        self.pt_place = {'pos': [50, -50, 10],  'roll': -80, 'pitch':  18, 'yaw': -64}

        self.R_zero = np.matrix([
            [0,  0, 1],
            [0, -1, 0],
            [1,  0, 0]
        ]) 

        self.traj1 = self.generate_trajectory(self.pt_home['pos'], self.pt_pick['pos'], 
                                         self.pt_home['roll'], self.pt_pick['roll'], 
                                         self.pt_home['pitch'], self.pt_pick['pitch'], 
                                         self.pt_home['yaw'], self.pt_pick['yaw'], 
                                         steps=50)
            
        self.traj2 = self.generate_trajectory(self.pt_pick['pos'], self.pt_lift['pos'], 
                                         self.pt_pick['roll'], self.pt_lift['roll'], 
                                         self.pt_pick['pitch'], self.pt_lift['pitch'], 
                                         self.pt_pick['yaw'], self.pt_lift['yaw'], 
                                         steps=50)
            
        self.traj3 = self.generate_trajectory(self.pt_lift['pos'], self.pt_place['pos'], 
                                         self.pt_lift['roll'], self.pt_place['roll'], 
                                         self.pt_lift['pitch'], self.pt_place['pitch'], 
                                         self.pt_lift['yaw'], self.pt_place['yaw'], 
                                         steps=50)
        
        self.full_trajectory = self.traj1 + self.traj2 + self.traj3

        if not self.full_trajectory:
            self.get_logger().error("Trajectory is empty — check that all waypoints are reachable!")
        else:
            self.get_logger().info(f"Trajectory ready: {len(self.full_trajectory)} steps.")

        self.index = 0


    def timer_callback(self):
        if not self.full_trajectory:
            return

        if self.index >= len(self.full_trajectory):
            self.index = 0  # loop trajectory

        angles, pos, vel, _, _ = self.full_trajectory[self.index]
        self.update_path_marker(pos)

        # Convert to radians before publishing to ROS
        # angles_rad = np.radians(angles_deg)
        self.publish_joint_states(angles, vel)

        self.index += 1

    
    def update_path_marker(self, pos):

        p = Point()
        p.x = pos[0] / 100
        p.y = pos[1] / 100
        p.z = pos[2] / 100

        self.path_points.append(p)

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "ee_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.015  # line thickness

        # color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = self.path_points

        self.marker_publisher.publish(marker)


    def publish_joint_states(self, angles, vel):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # These names MUST match the joint names in URDF exactly
        msg.name     = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']
        msg.position = angles
        msg.velocity = vel

        self.angles_publisher.publish(msg)

    def extract_coords(self, t1, t2, t3, t4, t5, t6):

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


    def new_R0_3(self,theta1, theta2, theta3):
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


    def return_cur_orientation(self, roll, pitch, yaw):

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

        cur_orientation = (Rz @ Ry @ Rx) @ self.R_zero

        return cur_orientation


    def inverse_kinematics(self, target_pos, target_orientation):
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

            R0_3_new = self.new_R0_3(theta1=theta1, theta2=theta2, theta3=theta3)
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

    def generate_trajectory(self, start_pos, end_pos, start_roll, end_roll,
                        start_pitch, end_pitch, start_yaw, end_yaw, steps):
        trajectory_data = []
        t_values = np.linspace(0, 1, steps)

        start_orientation = self.return_cur_orientation(start_roll, start_pitch, start_yaw)
        end_orientation = self.return_cur_orientation(end_roll, end_pitch, end_yaw)

        start_angles = self.inverse_kinematics(start_pos, start_orientation)
        end_angles = self.inverse_kinematics(end_pos, end_orientation)

        if start_angles[0] is None:
            print(f"ERROR: Start position {start_pos} is unreachable!")
            return []
        if end_angles[0] is None:
            print(f"ERROR: End position {end_pos} is unreachable!")
            return []

        for t in t_values:

            s = 10 * (t**3) - 15 * (t**4) + 6 * (t**5)
            v = 30*t**2 -60*t**3 + 30*t**4 # ds_dt

            cur_angles = []
            cur_vels = []

            for i in range(6):
                # interpolating joint angles
                cur_angle = start_angles[i] + s * (end_angles[i] - start_angles[i])
                cur_angles.append(cur_angle)

                cur_vel = v * (end_angles[i] - start_angles[i])
                cur_vels.append(cur_vel)

            cur_roll = start_roll + s * (end_roll - start_roll)
            cur_pitch = start_pitch + s * (end_pitch - start_pitch)
            cur_yaw = start_yaw + s * (end_yaw - start_yaw)
            cur_orientation = self.return_cur_orientation(cur_roll, cur_pitch, cur_yaw)

            t1, t2, t3, t4, t5, t6 = cur_angles

            _, _, _, _, _, _, _, _, p9, _ = self.extract_coords(t1, t2, t3, t4, t5, t6)

            cur_pos = [
                float(p9[0,0]),
                float(p9[1,0]),
                float(p9[2,0])
            ]

            if cur_angles[0] is not None:
                trajectory_data.append((cur_angles, cur_pos, cur_vels, cur_orientation, (cur_roll, cur_pitch, cur_yaw)))
            else:
                print(f"Skipping unreachable point at t={t:.3f}")

        return trajectory_data


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()