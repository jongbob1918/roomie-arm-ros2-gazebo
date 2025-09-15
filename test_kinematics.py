#!/usr/bin/env python3

import sys
import os
sys.path.append('/home/mac/dev_ws/roomiearm-ros2-gazebo/roomiearm_core')

from roomiearm_core.kinematics_solver import DirectTransformIK
import math

def test_forward_kinematics():
    """정기구학 테스트"""
    print("=" * 50)
    print("🔄 Forward Kinematics Test")
    print("=" * 50)

    ik_solver = DirectTransformIK()

    # 테스트 케이스들
    test_cases = [
        ([0.0, 0.0, 0.0, 0.0], "초기 자세"),
        ([0.0, -0.7, 1.0, 0.6], "관측 자세"),
        ([0.5, 0.3, -0.2, 0.1], "임의 자세 1"),
        ([math.pi/4, -math.pi/4, math.pi/3, -math.pi/6], "임의 자세 2"),
    ]

    for i, (joints, description) in enumerate(test_cases):
        try:
            pos, rot = ik_solver.forward_kinematics(joints)
            in_workspace = ik_solver.is_in_workspace(pos)

            print(f"Test {i+1}: {description}")
            print(f"  Joints: {[f'{j:.3f}' for j in joints]}")
            print(f"  Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            print(f"  Workspace: {'✅ Valid' if in_workspace else '❌ Invalid'}")
            print()

        except Exception as e:
            print(f"❌ Test {i+1} FAILED: {e}")
            print()

def test_inverse_kinematics():
    """역기구학 테스트"""
    print("=" * 50)
    print("🎯 Inverse Kinematics Test")
    print("=" * 50)

    ik_solver = DirectTransformIK()

    # 작업공간 내 목표 위치들
    test_positions = [
        ([0.15, 0.0, 0.25], "정면 중간 높이"),
        ([0.0, 0.15, 0.25], "좌측 중간 높이"),
        ([0.10, 0.10, 0.30], "대각선 위치"),
        ([0.20, 0.0, 0.15], "정면 낮은 위치"),
        ([0.05, 0.0, 0.35], "정면 높은 위치"),
    ]

    current_joints = [0.0, 0.0, 0.0, 0.0]

    for i, (target_pos, description) in enumerate(test_positions):
        try:
            print(f"IK Test {i+1}: {description}")
            print(f"  Target: {target_pos}")

            # 작업공간 검사
            if not ik_solver.is_in_workspace(target_pos):
                print(f"  ❌ Target outside workspace")
                print()
                continue

            # 역기구학 계산
            joint_solution = ik_solver.inverse_kinematics(target_pos, current_joints)
            print(f"  Solution: {[f'{j:.3f}' for j in joint_solution]}")

            # 검증: 정기구학으로 역계산
            verify_pos, _ = ik_solver.forward_kinematics(joint_solution)
            error = [(target_pos[j] - verify_pos[j])**2 for j in range(3)]
            error_magnitude = sum(error)**0.5

            print(f"  Verification: {[f'{p:.3f}' for p in verify_pos]}")
            print(f"  Error: {error_magnitude:.6f}m")
            print(f"  Status: {'✅ PASS' if error_magnitude < 0.01 else '❌ FAIL'}")
            print()

        except Exception as e:
            print(f"  ❌ FAILED: {e}")
            print()

def test_urdf_parsing():
    """URDF 파싱 테스트"""
    print("=" * 50)
    print("📄 URDF Parsing Test")
    print("=" * 50)

    try:
        ik_solver = DirectTransformIK()
        joint_chain = ik_solver.joint_chain

        print("✅ URDF parsing successful")
        print("Joint chain:")
        for joint_name, joint_info in joint_chain.items():
            print(f"  {joint_name}: xyz={joint_info['xyz']}, rpy={joint_info['rpy']}")
        print()

    except Exception as e:
        print(f"❌ URDF parsing failed: {e}")
        print()

def test_workspace_limits():
    """작업공간 제한 테스트"""
    print("=" * 50)
    print("🏢 Workspace Limits Test")
    print("=" * 50)

    ik_solver = DirectTransformIK()

    # 경계 근처 위치들 테스트
    test_cases = [
        ([0.05, 0.0, 0.25], "최소 반경"),
        ([0.34, 0.0, 0.25], "최대 반경"),
        ([0.15, 0.0, 0.05], "최소 높이"),
        ([0.15, 0.0, 0.50], "최대 높이"),
        ([0.04, 0.0, 0.25], "반경 밖 (너무 가까움)"),
        ([0.35, 0.0, 0.25], "반경 밖 (너무 멀음)"),
        ([0.15, 0.0, 0.04], "높이 밖 (너무 낮음)"),
        ([0.15, 0.0, 0.51], "높이 밖 (너무 높음)"),
    ]

    for pos, description in test_cases:
        in_workspace = ik_solver.is_in_workspace(pos)
        print(f"  {description}: {pos} → {'✅ Valid' if in_workspace else '❌ Invalid'}")

    print()

if __name__ == "__main__":
    print("🚀 RoomieArm Kinematics Test Suite")
    print()

    test_urdf_parsing()
    test_forward_kinematics()
    test_inverse_kinematics()
    test_workspace_limits()

    print("=" * 50)
    print("✅ Test suite completed!")