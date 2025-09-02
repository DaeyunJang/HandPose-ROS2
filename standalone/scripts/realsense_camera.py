from __future__ import annotations

from typing import Optional, Dict, Tuple
import numpy as np
import cv2

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

# === config 불러오기 ===
from hand_config_loader import get_config
_CFG = get_config()
CAMERA_CFG = _CFG["defaults"]["camera"]  # config.json의 defaults.camera 항목
Intrinsics = Dict[str, float]  # keys: fx, fy, cx, cy


class RealSenseCamera:
    """
    컬러/깊이 프레임 동기 획득 래퍼.
    """
    def __init__(
        self,
        width: int = CAMERA_CFG["width"],
        height: int = CAMERA_CFG["height"],
        fps: int = CAMERA_CFG["fps"],
        enable_color: bool = True,
        enable_depth: bool = True,
        align_to_color: bool = True,
    ) -> None:
        if rs is None:
            raise ImportError("pyrealsense2가 설치되어 있지 않습니다. `pip install pyrealsense2`")

        self.width: int = int(width)
        self.height: int = int(height)
        self.fps: int = int(fps)
        self.enable_color: bool = enable_color
        self.enable_depth: bool = enable_depth
        self.align_to_color: bool = align_to_color

        self.pipeline: rs.pipeline = rs.pipeline()
        self.config: rs.config = rs.config()

        if self.enable_color:
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        if self.enable_depth:
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)

        self._align: Optional[rs.align] = (
            rs.align(rs.stream.color) if (enable_color and enable_depth and align_to_color) else None
        )

        self._profile: Optional[rs.pipeline_profile] = None
        self.depth_scale_m_per_unit: Optional[float] = None
        self._depth_intrinsics: Optional[Intrinsics] = None

    def start(self) -> None:
        self._profile = self.pipeline.start(self.config)
        if self.enable_depth:
            depth_sensor: rs.sensor = self._profile.get_device().first_depth_sensor()
            self.depth_scale_m_per_unit = float(depth_sensor.get_depth_scale())
        else:
            self.depth_scale_m_per_unit = None

        if self.enable_depth:
            dstream: rs.video_stream_profile = (
                self._profile.get_stream(rs.stream.depth).as_video_stream_profile()
            )
            intr = dstream.get_intrinsics()
            self._depth_intrinsics = {
                "fx": float(intr.fx),
                "fy": float(intr.fy),
                "cx": float(intr.ppx),
                "cy": float(intr.ppy),
            }
        else:
            self._depth_intrinsics = None

    def get_frames(
        self,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[Intrinsics]]:
        frames: rs.composite_frame = self.pipeline.wait_for_frames()
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame() if self.enable_color else None
        depth_frame = frames.get_depth_frame() if self.enable_depth else None

        color_frame_bgr = (
            np.asanyarray(color_frame.get_data()) if color_frame is not None else None
        )

        if depth_frame is not None:
            scale = self.depth_scale_m_per_unit if self.depth_scale_m_per_unit is not None else 1.0
            depth_frame_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * scale
        else:
            depth_frame_m = None

        return color_frame_bgr, depth_frame_m, self._depth_intrinsics

    def stop(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()


if __name__ == "__main__":
    cam = RealSenseCamera(enable_color=True, enable_depth=False)
    try:
        cam.start()
        print(f"RealSense 카메라 시작됨 ({CAMERA_CFG['width']}x{CAMERA_CFG['height']} @ {CAMERA_CFG['fps']} FPS)")

        while True:
            color_frame, depth_frame, intrinsics = cam.get_frames()
            if color_frame is not None:
                cv2.imshow("Color", color_frame)
            if depth_frame is not None:
                depth_mm = (depth_frame * 1000).astype(np.uint16)
                depth_vis = cv2.convertScaleAbs(depth_mm, alpha=0.03)
                depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow("Depth", depth_colored)
            key = cv2.waitKey(1)
            if key == 27:
                break

    except Exception as e:
        print("오류 발생:", e)
    finally:
        cam.stop()
        cv2.destroyAllWindows()
        print("카메라 종료됨.")
