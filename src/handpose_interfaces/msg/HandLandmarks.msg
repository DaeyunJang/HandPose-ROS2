std_msgs/Header header

int8 id                       # 같은 프레임 내에서 손 인덱스
string label                   # "left" | "right"
float32 score                  # handedness score

uint32 width                   # 입력 이미지 W
uint32 height                  # 입력 이미지 H

# 21x3 을 1차원으로 평탄화 (length = 63)
float32[] landmarks_norm       # (x,y,z) normalized, z: mediapipe scale
float32[] landmarks_canon      # (x,y,z) canonical/pixel; x in [0,W], y in [0,H], z in [pixel of W]
float32[] landmarks_world      # (x,y,z) meter, origin=hand geometric center (없으면 빈 배열)

# 선택: multi-hands 정렬 유지용 인덱스(미사용시 -1)
int32 handed_index
