# demo_opencv.py
import threading
import traceback
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ====== 설정 ======
WIDTH, HEIGHT, FPS = 1280, 720, 30
MAX_NUM_HANDS = 4
DET_CONF, TRK_CONF = 0.6, 0.6

# 보기용 좌우 반전 (화면 미러링 효과)
HORIZONTAL_FLIP_VIEW = True

# 플롯 축 고정 (입력은 MediaPipe 정규화 좌표이므로 0..1 범위, wrist 변환해도 대략 비슷한 스케일)
USE_FIXED_LIMS = True
FIXED_LIMS = {
    "x": (-0.3, 0.3),
    "y": (-0.3, 0.3),
    "z": (-0.3, 0.3),
}

# FIXED_LIMS = {
#     "x": (-200, 200),
#     "y": (-200, 200),
#     "z": (-200, 200),
# }

# 손 라벨: "left" 또는 "right"
HAND_LABEL = "left"

# ====== MediaPipe / RealSense ======
import mediapipe as mp
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=MAX_NUM_HANDS,
                       min_detection_confidence=DET_CONF,
                       min_tracking_confidence=TRK_CONF)

import pyrealsense2 as rs
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

# ====== Landmark → Wrist 변환 클래스 ======
from landmark_to_handpose import LandmarkToHandPose

# 변환기(손목 프레임 계산기) — 뷰만 좌우반전할 거라서 flip_x=False 권장
pose_solver = LandmarkToHandPose(flip_x=False, hand_label=HAND_LABEL)

# ====== Plot 준비 ======
fig = plt.figure("Hand landmarks in WRIST frame")
ax = fig.add_subplot(111, projection='3d')
ax.set_title("MediaPipe → Wrist Frame (no depth)")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.set_box_aspect([1,1,1])

def apply_fixed_lims():
    if USE_FIXED_LIMS:
        ax.set_xlim(*FIXED_LIMS["x"])
        ax.set_ylim(*FIXED_LIMS["y"])
        ax.set_zlim(*FIXED_LIMS["z"])

# 초기 고정
apply_fixed_lims()

# 손가락 연결 (MediaPipe 인덱스)
FINGERS = {
    'thumb': [1, 2, 3, 4],
    'index': [5, 6, 7, 8],
    'middle': [9, 10, 11, 12],
    'ring': [13, 14, 15, 16],
    'pinky': [17, 18, 19, 20],
}
PALM_CONNECTIONS = [(0,1),(0,5),(0,9),(0,13),(0,17)]
FINGER_COLORS = {'thumb':'r','index':'g','middle':'b','ring':'c','pinky':'m'}

# 아티팩트 핸들
finger_scatters = {
    finger: ax.scatter([], [], [], c=FINGER_COLORS[finger], s=20, label=finger)
    for finger in FINGERS.keys()
}
finger_lines = {
    finger: [ax.plot([], [], [], c=FINGER_COLORS[finger])[0] for _ in range(len(FINGERS[finger]) - 1)]
    for finger in FINGERS.keys()
}
palm_lines = [ax.plot([], [], [], c='gray', linestyle='--')[0] for _ in PALM_CONNECTIONS]
ax.legend()

# ====== 공유 상태 ======
latest_coords_input = None  # (21,3) in input coords (mediapipe normalized)
lock = threading.Lock()
running = True

# ====== 캡처/검출 쓰레드 ======
def capture_and_detect():
    global latest_coords_input, running
    try:
        pipeline.start(config)
        print("[run] RealSense started")
    except Exception as e:
        print("[run][fatal] RealSense start failed:", e)
        traceback.print_exc()
        sys.exit(1)

    try:
        while running:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            if HORIZONTAL_FLIP_VIEW:
                frame = cv2.flip(frame, 1)

            # MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(rgb)

            coords = None
            if res.multi_hand_landmarks:
                lm = res.multi_hand_landmarks[0]  # 첫 손만
                # 디버그 시각화(2D)
                mp_drawing.draw_landmarks(frame, lm, mp_hands.HAND_CONNECTIONS)
                coords = np.array([(p.x, p.y, p.z) for p in lm.landmark], dtype=np.float32)

            with lock:
                latest_coords_input = coords

            cv2.imshow("RealSense color (debug)", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                running = False
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[run] capture thread stopped")

# ====== 3D 업데이트 ======
def update_plot(_):
    with lock:
        coords_in = latest_coords_input

    if coords_in is None:
        # 아무 손도 없을 때는 축만 유지
        ax.set_autoscale_on(False)
        apply_fixed_lims()
        return []

    # 1) 입력 좌표계(정규화) → wrist 좌표계로 변환
    try:
        pose_solver.update_landmarks(coords_in)              # (21,3) in input coords
        frames = pose_solver.compute(label=HAND_LABEL)       # wrist 프레임 구성
        lm_w = frames.landmarks_wrist                        # (21,3) in wrist coords
    except Exception as e:
        # wrist 변환이 이상하면 여기서 잡힘
        print("[run][wrist-xform-error]", e)
        traceback.print_exc()
        return []

    # 2) 그리기(매 프레임 cla 후 축 고정 다시)
    ax.cla()
    # ax.set_title("MediaPipe → Wrist Frame (no depth)")
    # ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    # ax.grid(True)
    # ax.set_box_aspect([1,1,1])
    # ax.set_autoscale_on(False)

    # 포인트
    ax.scatter(lm_w[:,0], lm_w[:,1], lm_w[:,2], s=10, c='k')

    # 손가락 라인 + 손바닥 연결
    for finger, idxs in FINGERS.items():
        pts = lm_w[idxs, :]
        ax.plot(pts[:,0], pts[:,1], pts[:,2], '-', c=FINGER_COLORS[finger])
        # wrist(0)→mcp 연결선
        ax.plot([lm_w[0,0], pts[0,0]], [lm_w[0,1], pts[0,1]], [lm_w[0,2], pts[0,2]], '--', c=FINGER_COLORS[finger])

    for i, (s, e) in enumerate(PALM_CONNECTIONS):
        ax.plot([lm_w[s,0], lm_w[e,0]],
                [lm_w[s,1], lm_w[e,1]],
                [lm_w[s,2], lm_w[e,2]], c='gray', linestyle='--')

    # 각 조인트 프레임 축(원하면 주석 해제)
    for (finger, jname), T in frames.T_wrist2joint.items():
        o = T[:3, 3]; X=T[:3,0]; Y=T[:3,1]; Z=T[:3,2]
        ax.quiver(o[0], o[1], o[2], X[0], X[1], X[2], length=0.02, color='r', linewidth=1.2)
        ax.quiver(o[0], o[1], o[2], Y[0], Y[1], Y[2], length=0.02, color='g', linewidth=1.2)
        ax.quiver(o[0], o[1], o[2], Z[0], Z[1], Z[2], length=0.02, color='b', linewidth=1.2)

    # 축 고정(항상 마지막에)
    # apply_fixed_lims()
    return []

# ====== 실행 ======
if __name__ == "__main__":
    t = threading.Thread(target=capture_and_detect, daemon=True)
    t.start()
    ani = FuncAnimation(fig, update_plot, interval=10)
    try:
        plt.show()
    finally:
        running = False
        t.join(timeout=1.0)
        print("[run] exited")
