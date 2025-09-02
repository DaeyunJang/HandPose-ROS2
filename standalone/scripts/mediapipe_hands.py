from __future__ import annotations

from typing import Optional, Tuple, Dict, List
import numpy as np
from numpy.typing import NDArray
import cv2

try:
    import mediapipe as mp
except ImportError:
    mp = None


class MediaPipeHandEstimator:
    """
    RGB 이미지를 넣으면 손 랜드마크를 반환.

    Notes
    -----
    - MediaPipe Hands는 (x, y) ∈ [0, 1] 정규화(이미지 폭/높이 기준), z는 카메라 앞으로 음수인 상대값(픽셀이 아님).
    - 여기서는 정규화 좌표를 픽셀 기준의 'canonical' 좌표로 스케일링한 값을 추가로 반환한다.
    """

    def __init__(
        self,
        static_image_mode: bool = False,
        max_num_hands: int = 4,
        min_detection_confidence: float = 0.6,
        min_tracking_confidence: float = 0.6,
    ) -> None:
        if mp is None:
            raise ImportError("mediapipe가 설치되어 있지 않습니다. `pip install mediapipe`")

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=static_image_mode,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.image_with_landmarks: NDArray[np.int8] = None

    def detect(
            self,
            bgr: NDArray[np.uint8]
    ) -> Tuple[
        bool,
        List[Dict[str, object]],    # handedness_list
        List[NDArray[np.float32]],  # lm_norm_list
        List[NDArray[np.float32]],  # lm_canon_list
    ]:
        """
        Args:
            bgr: (H,W,3) BGR image (OpenCV)

        Returns:
            success: bool
            lm_norm_list:  [ (21,3) float32, ... ]       # 정규화 좌표 (x,y∈[0,1], z=상대)
            lm_canon_list: [ (21,3) float32, ... ]       # 픽셀 스케일 좌표 (x_px, y_px, z_scaled)
            handed_list:   [ {"index":int,"score":float,"label":str,"label_lower":str}, ... ]
                          # MediaPipe의 multi_handedness를 손별로 파싱 (label: "Left"/"Right")
        """
        if bgr is None:
            return False, [], [], []

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return False, [], [], []

        h, w = bgr.shape[:2]

        lm_norm_list = []
        lm_canon_list = []
        handedness_list = []

        # MediaPipe는 multi_hand_landmarks[i] 와 multi_handedness[i]가 같은 손을 의미
        for i, lms in enumerate(res.multi_hand_landmarks):
            lm_norm = np.array([[lm.x, lm.y, lm.z] for lm in lms.landmark], dtype=np.float32)
            lm_canon = self.norm_to_canonical(lm_norm=lm_norm, width=w, height=h, z_scale="width")

            lm_norm_list.append(lm_norm)
            lm_canon_list.append(lm_canon)

            self.image_with_landmarks = bgr
            self.mp_drawing.draw_landmarks(self.image_with_landmarks, lms, self.mp_hands.HAND_CONNECTIONS)

            if res.multi_handedness and i < len(res.multi_handedness):
                cls = res.multi_handedness[i].classification[0]  # 보통 하나만 존재
                handedness = {
                    "index": int(cls.index),
                    "score": float(cls.score),
                    "label": str(cls.label),
                    # "label_lower": str(cls.label).lower(),  # "left" | "right"
                }
            else:
                handedness = {"index": -1, "score": 0.0, "label": "Unknown", "label_lower": "unknown"}

            handedness_list.append(handedness)

        return True, handedness_list, lm_norm_list, lm_canon_list

    @staticmethod
    def norm_to_canonical(
        lm_norm: NDArray[np.float32],
        width: int,
        height: int,
        z_scale: float | str | None = "width",  # "width" | "height" | float | None
        as_int: bool = False,
    ) -> NDArray[np.float32] | NDArray[np.int32]:
        """
        MediaPipe 정규화 랜드마크(lm_norm: (21,3), x/y∈[0,1], z=상대)를
        픽셀 스케일의 canonical 좌표로 변환.

        - x: [0,1] → [0, width)
        - y: [0,1] → [0, height)
        - z: 상대값을 z_scale로 스케일 (기본: width). 절대깊이가 필요하면 depth 사용 권장.

        Parameters
        ----------
        lm_norm : (21, 3) float32
        width : int
        height : int
        z_scale : {"width","height"} | float | None
        as_int : bool
            True면 정수 픽셀 좌표 반환

        Returns
        -------
        (21, 3) float32 또는 int32
        """
        assert lm_norm.shape == (21, 3), f"Expected (21,3), got {lm_norm.shape}"

        canon = lm_norm.astype(np.float32).copy()

        # x, y 스케일
        # width, height에 float()을 사용한 이유는 명시적으로 결과값이 float임을 보여주기 위해서
        canon[:, 0] = canon[:, 0] * float(width)
        canon[:, 1] = canon[:, 1] * float(height)

        # z 스케일링
        if z_scale is None:
            pass
        elif z_scale == "width":
            canon[:, 2] = canon[:, 2] * float(width)
        elif z_scale == "height":
            canon[:, 2] = canon[:, 2] * float(height)
        elif isinstance(z_scale, (int, float)):
            canon[:, 2] = canon[:, 2] * float(z_scale)
        else:
            raise ValueError("z_scale must be 'width', 'height', a number, or None")

        if as_int:
            # 반올림 후 정수 캐스팅
            return np.rint(canon).astype(np.int32)
        return canon

    def get_images_with_landmarks(self) -> NDArray[np.int8]:
        return self.image_with_landmarks

    @staticmethod
    def backproject_to_3d(
        lm_px: NDArray[np.int32],
        depth_m: Optional[NDArray[np.float32]],
        intrinsics: Optional[Dict[str, float]],
    ) -> NDArray[np.float32]:
        """
        픽셀 좌표와 깊이(m), 내참수로 3D 포인트(X,Y,Z)[m] 산출 (핀홀 역투영)

        Parameters
        ----------
        lm_px : (21, 2) int32
            픽셀 좌표 (x_px, y_px) -- 인덱싱에 사용되므로 정수 권장
        depth_m : (H, W) float32 | None
            깊이 이미지(미터). None이면 Z=0으로 fallback
        intrinsics : dict | None
            {"fx","fy","cx","cy"} (depth/카메라 스트림 기준)

        Returns
        -------
        pts_3d : (21, 3) float32
        """
        if depth_m is None or intrinsics is None:
            pts = np.zeros((21, 3), dtype=np.float32)
            pts[:, 0] = lm_px[:, 0].astype(np.float32)
            pts[:, 1] = lm_px[:, 1].astype(np.float32)
            pts[:, 2] = 0.0
            return pts

        fx: float = float(intrinsics["fx"])
        fy: float = float(intrinsics["fy"])
        cx: float = float(intrinsics["cx"])
        cy: float = float(intrinsics["cy"])

        # 정수 인덱스 (보장)
        x_px: NDArray[np.int32] = lm_px[:, 0]
        y_px: NDArray[np.int32] = lm_px[:, 1]

        # 깊이 샘플링 (미터)
        z: NDArray[np.float32] = depth_m[y_px, x_px].astype(np.float32)

        X: NDArray[np.float32] = ((x_px.astype(np.float32) - cx) * z) / fx
        Y: NDArray[np.float32] = ((y_px.astype(np.float32) - cy) * z) / fy
        Z: NDArray[np.float32] = z

        pts: NDArray[np.float32] = np.stack([X, Y, Z], axis=1)
        # NaN/Inf 보호
        pts = np.nan_to_num(pts, nan=0.0, posinf=0.0, neginf=0.0)
        return pts



if __name__ == "__main__":
    import argparse
    import os

    if mp is None:
        raise ImportError("mediapipe가 설치되어 있지 않습니다. `pip install mediapipe`")


    bgr = cv2.imread('hsh.jpg', cv2.IMREAD_COLOR)


    # parser = argparse.ArgumentParser(description="Test MediaPipeHandEstimator on a single image.")
    # parser.add_argument("--img", type=str, required=True, help="Input image path")
    # parser.add_argument("--flip", action="store_true", help="Horizontally flip input before inference")
    # parser.add_argument("--save", type=str, default=None, help="Path to save visualization (optional)")
    # args = parser.parse_args()
    #
    # if not os.path.isfile(args.img):
    #     raise FileNotFoundError(f"Image not found: {args.img}")
    #
    # # 이미지 로드 (BGR)
    # bgr = cv2.imread(args.img, cv2.IMREAD_COLOR)
    # if bgr is None:
    #     raise RuntimeError(f"Failed to read image: {args.img}")
    #
    # if args.flip:
    #     bgr = cv2.flip(bgr, 1)

    estimator = MediaPipeHandEstimator(
        static_image_mode=True,          # 단일 이미지일 때는 True가 유리
        max_num_hands=4,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6
    )

    ok, handedness, lm_norm, lm_canon = estimator.detect(bgr)
    vis = bgr.copy()

    if ok:
        # 시각화: canonical 좌표를 정수로 변환해서 원/선으로 표시
        h, w = vis.shape[:2]
        lm_canon_px = estimator.norm_to_canonical(
            lm_norm=lm_norm, width=w, height=h, z_scale=None, as_int=True
        )  # z는 사용 안 하므로 z_scale=None

        # 원 그리기
        for i, (x, y, _) in enumerate(lm_canon_px):
            cv2.circle(vis, (int(x), int(y)), 3, (0, 255, 0), -1)
            cv2.putText(vis, str(i), (int(x) + 4, int(y) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_AA)

        # 손가락별 간선(간단 버전)
        fingers = {
            "thumb":  [1, 2, 3, 4],
            "index":  [5, 6, 7, 8],
            "middle": [9, 10, 11, 12],
            "ring":   [13, 14, 15, 16],
            "pinky":  [17, 18, 19, 20],
        }
        palm_connections = [(0,1), (0,5), (0,9), (0,13), (0,17)]

        def _p(i):
            return (int(lm_canon_px[i, 0]), int(lm_canon_px[i, 1]))

        # 손바닥 라인
        for s, e in palm_connections:
            cv2.line(vis, _p(s), _p(e), (200, 200, 200), 1, cv2.LINE_AA)

        # 손가락 라인
        for idxs in fingers.values():
            for s, e in zip(idxs[:-1], idxs[1:]):
                cv2.line(vis, _p(s), _p(e), (0, 255, 255), 1, cv2.LINE_AA)

        title = "Hand Landmarks (canonical overlaid)"
    else:
        title = "No hand detected"

    # 결과 표시 및 저장
    cv2.imshow(title, vis)
    if args.save:
        cv2.imwrite(args.save, vis)
        print(f"Saved to: {args.save}")

    print("Press any key (window focus) to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
