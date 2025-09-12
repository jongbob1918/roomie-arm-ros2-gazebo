import cv2
import numpy as np
import pathlib
import os

# ====================================================================
# --- 설정 변수 (사용자 환경에 맞게 반드시 수정하세요) ---
# ====================================================================

# 1. 체커보드 설정 (내부 코너의 개수)
# 예: 가로 10칸, 세로 7칸 체커보드 -> (9, 6)
CHECKERBOARD_SIZE = (19, 14)

# 2. 체커보드 한 칸의 실제 크기 (미터 단위)
SQUARE_SIZE_M = 0.011

# 3. 캘리브레이션에 사용할 카메라 장치 ID
CAMERA_DEVICE_ID = 12

# 4. 캘리브레이션 이미지 및 결과 파일을 저장할 경로
# 스크립트가 있는 폴더 아래에 'calibration_data' 폴더를 생성하고 그 안에 저장합니다.
OUTPUT_DIR = pathlib.Path(__file__).parent / "calibration_data"
IMAGE_SAVE_DIR = OUTPUT_DIR / "images"
OUTPUT_FILE_PATH = OUTPUT_DIR / "camera_params.npz"

# 5. 캘리브레이션에 사용할 최소 이미지 개수
MIN_IMAGES = 20

# ====================================================================

def main():
    """메인 캘리브레이션 함수"""
    print("📷 카메라 캘리브레이션을 시작합니다.")
    print("--------------------------------------------------")
    print(f"1. 체커보드 내부 코너: {CHECKERBOARD_SIZE}")
    print(f"2. 사각형 크기: {SQUARE_SIZE_M * 1000:.2f} mm")
    print("--------------------------------------------------")

    # 저장 폴더 생성
    IMAGE_SAVE_DIR.mkdir(parents=True, exist_ok=True)

    # 3D 점 (0,0,0), (1,0,0), (2,0,0) ...., (9,5,0) 등을 준비합니다.
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
    objp = objp * SQUARE_SIZE_M

    # 모든 이미지에 대한 3D 점과 2D 점을 저장할 배열
    objpoints = []  # 3D 점
    imgpoints = []  # 2D 점

    # 카메라 열기
    cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
    if not cap.isOpened():
        print(f"오류: 카메라 ID {CAMERA_DEVICE_ID}를 열 수 없습니다.")
        return

    print("\n[촬영 안내]")
    print(f" - 체커보드를 다양한 각도, 위치, 거리에서 보여주세요.")
    print(f" - [{MIN_IMAGES}장 이상] 촬영하는 것을 권장합니다.")
    print(" - [스페이스바]: 현재 화면 캡처 및 코너 검출")
    print(" - [Enter]: 캘리브레이션 시작 (이미지 촬영 종료)")
    print(" - [q]: 종료")
    print("--------------------------------------------------")

    captured_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("오류: 카메라에서 프레임을 읽을 수 없습니다.")
            break

        # 화면에 안내 텍스트 표시
        text = f"Captured: {captured_count}/{MIN_IMAGES} | Press 'space' to capture, 'enter' to calibrate, 'q' to quit"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Camera Calibration - Press Space to Capture', frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print("캘리브레이션을 중단합니다.")
            cap.release()
            cv2.destroyAllWindows()
            return

        elif key == ord(' '):  # 스페이스바를 눌렀을 때
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 체커보드 코너 찾기
            ret_corners, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

            if ret_corners:
                captured_count += 1
                objpoints.append(objp)

                # 서브픽셀 단위로 코너 위치를 더 정확하게 찾음
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                
                # 이미지 파일 저장
                img_path = str(IMAGE_SAVE_DIR / f'calib_{captured_count:02d}.png')
                cv2.imwrite(img_path, frame)
                
                # 찾은 코너를 화면에 그려서 보여줌 (시각적 피드백)
                cv2.drawChessboardCorners(frame, CHECKERBOARD_SIZE, corners2, ret_corners)
                cv2.imshow('Camera Calibration - Press Space to Capture', frame)
                print(f"✅ 성공! {captured_count}번째 이미지 저장 완료. ({img_path})")
                cv2.waitKey(500) # 0.5초 동안 결과 보여주기
            else:
                print("❌ 실패: 현재 화면에서 체커보드를 찾을 수 없습니다. 다른 각도에서 시도하세요.")

        elif key == 13:  # Enter 키를 눌렀을 때
            if len(imgpoints) < MIN_IMAGES:
                print(f"이미지가 {len(imgpoints)}장밖에 없습니다. {MIN_IMAGES}장 이상 촬영해주세요.")
                continue
            
            print(f"\n{len(imgpoints)}장의 이미지로 캘리브레이션을 시작합니다...")
            break

    cap.release()
    cv2.destroyAllWindows()

    if not imgpoints:
        print("캘리브레이션할 이미지가 없어 종료합니다.")
        return

    # 카메라 캘리브레이션 실행
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if not ret:
        print("캘리브레이션에 실패했습니다.")
        return

    print("\n🎉 캘리브레이션 성공!")
    print("--------------------------------------------------")
    print(f"Reprojection Error: {ret:.4f}")
    print(f"(이 값이 0.5 이하면 매우 좋습니다.)")
    print("\nCamera Matrix (mtx):\n", mtx)
    print("\nDistortion Coefficients (dist):\n", dist)
    print("--------------------------------------------------")

    # 결과 파일 저장
    np.savez(OUTPUT_FILE_PATH, mtx=mtx, dist=dist)
    print(f"결과가 '{OUTPUT_FILE_PATH}' 파일로 저장되었습니다.")


    # --- 실시간 왜곡 보정 검증 ---
    print("\n[검증 안내]")
    print(" - 왜곡 보정 결과를 실시간으로 확인합니다.")
    print(" - 'Undistorted' 창에서 직선(책상 모서리 등)이 곧게 보이는지 확인하세요.")
    print(" - [q]: 모든 과정 종료")
    print("--------------------------------------------------")

    cap = cv2.VideoCapture(CAMERA_DEVICE_ID)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 왜곡 보정 적용
        undistorted_frame = cv2.undistort(frame, mtx, dist, None, None)
        
        # 두 영상을 가로로 합쳐서 보여주기
        combined_view = np.hstack((frame, undistorted_frame))
        cv2.putText(combined_view, "Original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined_view, "Undistorted", (frame.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Verification: Original vs Undistorted', combined_view)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    print("프로그램을 종료합니다.")


if __name__ == '__main__':
    main()