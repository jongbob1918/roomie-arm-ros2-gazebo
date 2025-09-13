# roomie_arm_control/kinematics_solver.py

import numpy as np
from ikpy.chain import Chain
from . import config # config.py 임포트

class KinematicsSolver:
    """ikpy를 사용하여 Inverse Kinematics 계산을 수행합니다."""
    
    def __init__(self):
        # arm_description 패키지의 xacro 파일 사용
        from ament_index_python.packages import get_package_share_directory
        import os
        
        try:
            # arm_description 패키지에서 URDF 로드
            arm_description_path = get_package_share_directory('arm_description')
            urdf_file = os.path.join(arm_description_path, 'urdf', 'roomie_4dof.xacro')
            
            # xacro를 URDF로 변환
            import subprocess
            import tempfile
            
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp_file:
                result = subprocess.run(['xacro', urdf_file], capture_output=True, text=True)
                if result.returncode == 0:
                    tmp_file.write(result.stdout)
                    tmp_urdf_path = tmp_file.name
                else:
                    print(f"xacro 변환 실패: {result.stderr}")
                    # fallback to roomie_ac URDF
                    tmp_urdf_path = str(config.URDF_FILE)
                    
            self.chain = Chain.from_urdf_file(tmp_urdf_path, active_links_mask=config.ACTIVE_LINKS_MASK)
            
            # 임시 파일 정리
            if tmp_urdf_path != str(config.URDF_FILE):
                os.unlink(tmp_urdf_path)
                
        except Exception as e:
            print(f"arm_description URDF 로드 실패, roomie_ac URDF 사용: {e}")
            # fallback to original URDF
            self.chain = Chain.from_urdf_file(str(config.URDF_FILE), active_links_mask=config.ACTIVE_LINKS_MASK)
            
        self.active_links_mask = config.ACTIVE_LINKS_MASK

    def _get_full_joints(self, active_joints_rad):
        """활성 관절 배열(len=4)을 전체 관절 배열(len=6)로 변환합니다."""
        full_joints = np.zeros(len(self.chain.links))
        full_joints[self.active_links_mask] = active_joints_rad
        return full_joints

    def solve_ik(self, target_pos, target_orientation, current_active_angles_rad):
        """
        [수정됨] 목표 3D 위치와 '방향 행렬 (또는 None)'에 대한 IK 솔루션을 계산합니다.
        target_orientation이 None이면 ikpy가 방향을 제한하지 않습니다.
        """
        if config.DEBUG:
            print("\n--- IK 계산 시작 ---")
            print(f"  - 목표 좌표 (m): {np.round(target_pos, 4)}")
            print(f"  - 목표 방향: {'자동' if target_orientation is None else '지정됨'}")
            print(f"  - 현재 활성 관절 각도 (rad): {np.round(current_active_angles_rad, 4)}")

        q_full_seed = self._get_full_joints(current_active_angles_rad)
        
        try:
            q_solution_all = self.chain.inverse_kinematics(
                target_position=target_pos,
                # [수정] target_orientation 인자를 그대로 전달 (None이 될 수 있음)
                target_orientation=target_orientation,
                orientation_mode=None, # [수정] 특정 축 고정 대신 전체 방향 행렬을 사용하므로 None으로 설정
                initial_position=q_full_seed,
                max_iter=config.IK_MAX_ITERATIONS
            )
        except Exception as e:
            if config.DEBUG:
                print(f"❌ IK 계산 중 오류 발생: {e}")
            return None
        
        # ... (이후의 오차 검사 및 반환 로직은 기존과 동일) ...
        final_pos = self.chain.forward_kinematics(q_solution_all)[:3, 3]
        error = np.linalg.norm(final_pos - target_pos)

        active_solution_rad = q_solution_all[self.active_links_mask]
        if config.DEBUG:
            print(f"  - IK 결과 (활성, rad): {np.round(active_solution_rad, 4)}")
            print(f"  - 최종 도달 좌표 (m): {np.round(final_pos, 4)}")
            print(f"  - 오차 (m): {error:.6f}")

        if error > config.IK_TOLERANCE_M:
            if config.DEBUG:
                print(f"⚠️ IK 오차가 허용 범위를 초과합니다.")
            return None 
        
        for i, angle_rad in enumerate(active_solution_rad):
            if not (config.JOINT_LIMIT_RAD[i][0] <= angle_rad <= config.JOINT_LIMIT_RAD[i][1]):
                if config.DEBUG:
                    print(f"🚫 관절 {i+1}의 IK 결과 각도가 제한을 벗어납니다.")
                return None

        return active_solution_rad