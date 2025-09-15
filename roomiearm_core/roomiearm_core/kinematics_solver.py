import rclpy
import numpy as np
import math
from scipy.optimize import minimize
from rcl_interfaces.srv import GetParameters
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R


JOINT_LIMITS_MIN = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2]
JOINT_LIMITS_MAX = [math.pi/2, math.pi/2, math.pi/2, math.pi/2]


WORKSPACE_RADIUS_MIN = 0.05  # 5cm
WORKSPACE_RADIUS_MAX = 0.34  # 34cm (
WORKSPACE_HEIGHT_MIN = 0.05  # 5cm
WORKSPACE_HEIGHT_MAX = 0.50  # 50cm

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']


class URDFKinematicsParser:
    def __init__(self):
        self.joint_chain = self.parse_urdf_from_ros_param()

    def parse_urdf_from_ros_param(self):
        """robot_state_publisher URDF retrieval"""
        try:
            node = rclpy.create_node('temp_urdf_reader')
            client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')

            if not client.wait_for_service(timeout_sec=3.0):
                raise Exception("robot_state_publisher service not available")

            request = GetParameters.Request()
            request.names = ['robot_description']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            urdf_string = future.result().values[0].string_value
            node.destroy_node()

            return self.extract_joint_chain_from_urdf(urdf_string)
        except Exception as e:
            print(f"URDF parsing failed: {e}, using fallback")
            return self.get_fallback_joint_chain()

    def extract_joint_chain_from_urdf(self, urdf_string):
        """URDF XML parsing"""
        try:
            root = ET.fromstring(urdf_string)
            joint_chain = {}

            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                if joint_name in JOINT_NAMES + ['ee_joint']:
                    origin = joint.find('origin')
                    axis = joint.find('axis')

                    xyz = [0, 0, 0]
                    rpy = [0, 0, 0]
                    axis_vec = [0, 0, 1]

                    if origin is not None:
                        if origin.get('xyz'):
                            xyz = [float(x) for x in origin.get('xyz').split()]
                        if origin.get('rpy'):
                            rpy = [float(x) for x in origin.get('rpy').split()]

                    if axis is not None and axis.get('xyz'):
                        axis_vec = [float(x) for x in axis.get('xyz').split()]

                    joint_chain[joint_name] = {
                        'xyz': xyz,
                        'rpy': rpy,
                        'axis': axis_vec if joint.get('type') != 'fixed' else None
                    }

            # ee_joint fallback
            if 'ee_joint' not in joint_chain:
                joint_chain['ee_joint'] = {'xyz': [0.01, 0, 0.092], 'rpy': [0, 0, 0], 'axis': None}

            return joint_chain
        except Exception as e:
            print(f"URDF XML parsing failed: {e}")
            return self.get_fallback_joint_chain()

    def get_fallback_joint_chain(self):
        """Fallback joint chain (URDF parsing failed)"""
        return {
            'joint_1': {'xyz': [0, 0, 0.0814], 'rpy': [0, 0, 0], 'axis': [0, 0, 1]},
            'joint_2': {'xyz': [-0.02, 0, 0.015], 'rpy': [0, 0, 0], 'axis': [0, -1, 0]},
            'joint_3': {'xyz': [-0.0021, 0, 0.1035], 'rpy': [0, 0, 0], 'axis': [0, 1, 0]},
            'joint_4': {'xyz': [-0.002, 0, 0.1275], 'rpy': [0, 0, 0], 'axis': [0, -1, 0]},
            'ee_joint': {'xyz': [0.01, 0, 0.092], 'rpy': [0, 0, 0], 'axis': None}
        }


class DirectTransformIK:
    def __init__(self):
        self.urdf_parser = URDFKinematicsParser()
        self.joint_chain = self.urdf_parser.joint_chain

    def create_transform_matrix(self, xyz, rpy, axis, angle):
        """Transformation matrix creation"""
        T = np.eye(4)

        # Translation
        T[:3, 3] = xyz

        # Static rotation from RPY
        if any(rpy):
            r_static = R.from_euler('xyz', rpy)
            T[:3, :3] = r_static.as_matrix()

        # Joint rotation
        if axis is not None:
            axis_norm = np.array(axis) / np.linalg.norm(axis)
            r_joint = R.from_rotvec(axis_norm * angle)
            T[:3, :3] = T[:3, :3] @ r_joint.as_matrix()

        return T

    def forward_kinematics(self, joint_angles):
        """Forward Kinematics: URDF transformation matrix"""
        T = np.eye(4)
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'ee_joint']

        for i, joint_name in enumerate(joint_names):
            if joint_name not in self.joint_chain:
                continue

            joint_info = self.joint_chain[joint_name]
            angle = joint_angles[i] if i < len(joint_angles) and joint_name != 'ee_joint' else 0

            T_joint = self.create_transform_matrix(
                joint_info['xyz'], joint_info['rpy'], joint_info['axis'], angle
            )
            T = T @ T_joint

        return T[:3, 3], T[:3, :3]

    def inverse_kinematics(self, target_position, current_joints):
        """IK İ (0XY + Xt 1)"""
        try:
            return self.geometric_ik(target_position, current_joints)
        except Exception as e:
            print(f"Geometric IK failed: {e}, trying numerical")
            return self.numerical_ik(target_position, current_joints)

    def geometric_ik(self, target_position, current_joints):
        """Geometric IK"""
        x, y, z = target_position

        # Joint 1: Base rotation
        theta1 = math.atan2(y, x)

        # 2D 평면에서의 거리
        r = math.sqrt(x**2 + y**2)

        # Link lengths from URDF
        base_height = self.joint_chain['joint_1']['xyz'][2]  # 0.0814
        L1 = self.joint_chain['joint_2']['xyz'][2]  # 0.015
        L2 = self.joint_chain['joint_3']['xyz'][2]  # 0.1035
        L3 = self.joint_chain['joint_4']['xyz'][2]  # 0.1275
        L4 = self.joint_chain['ee_joint']['xyz'][2]  # 0.092

        # ��1 ��
        z_corrected = z - base_height - L1
        target_dist = math.sqrt(r**2 + z_corrected**2)
        max_reach = L2 + L3 + L4

        if target_dist > max_reach:
            raise ValueError(f"Target unreachable: {target_dist:.3f}m > {max_reach:.3f}m")

        # Law of cosines
        cos_theta3 = (target_dist**2 - L2**2 - (L3 + L4)**2) / (2 * L2 * (L3 + L4))
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)

        theta3 = math.acos(cos_theta3)

        alpha = math.atan2(z_corrected, r)
        beta = math.acos((L2**2 + target_dist**2 - (L3 + L4)**2) / (2 * L2 * target_dist))

        theta2 = alpha - beta
        theta4 = -(theta2 + theta3)  # End effector rotation

        result = [theta1, theta2, theta3, theta4]

        # Joint limit checking
        for i, angle in enumerate(result):
            if not (JOINT_LIMITS_MIN[i] <= angle <= JOINT_LIMITS_MAX[i]):
                raise ValueError(f"Joint {i+1} limit exceeded: {angle:.3f}")

        return result

    def numerical_ik(self, target_position, current_joints):
        """Numerical IK"""
        def objective(joint_angles):
            pos, _ = self.forward_kinematics(joint_angles)
            return np.linalg.norm(pos - np.array(target_position))

        result = minimize(
            objective, current_joints,
            bounds=[(JOINT_LIMITS_MIN[i], JOINT_LIMITS_MAX[i]) for i in range(4)],
            method='L-BFGS-B'
        )

        if result.success and result.fun < 0.01:
            return result.x.tolist()
        else:
            raise ValueError(f"Numerical IK failed: error={result.fun:.6f}m")

    def is_in_workspace(self, position):
        """Workspace check"""
        x, y, z = position
        radius = math.sqrt(x**2 + y**2)
        return (WORKSPACE_RADIUS_MIN <= radius <= WORKSPACE_RADIUS_MAX and
                WORKSPACE_HEIGHT_MIN <= z <= WORKSPACE_HEIGHT_MAX)

