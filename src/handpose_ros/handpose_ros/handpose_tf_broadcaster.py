#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import SetParametersResult

from handpose_interfaces.msg import Hands
from handpose_ros.landmark_to_handpose import LandmarkToHandPose
from handpose_ros.algebra_utils import *  # rotm_to_quat 등 사용
from handpose_ros.hand_config_loader import get_config, build_finger_joint_map


class HandPoseTFNode(Node):
    def __init__(self):
        super().__init__('handpose_tf_broadcaster')

        # ------------------------ 파라미터 ------------------------
        self.declare_parameter('hands_topic', 'hands/detections')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')    # TF name

        # 어떤 TF들을 퍼블리시할지
        self.declare_parameter('tf.norm.enable', True)
        self.declare_parameter('tf.canonical.enable', True)

        # canon_norm: base 스케일만 적용 (자동 스케일 없음)
        self.declare_parameter('tf.canonical_norm.enable', True)
        self.declare_parameter('tf.canonical_norm.scale', 1/1280)  # base scale

        # real world_absolute_scale (양손 독립 Wrist→MCP 절대 스케일)
        # ※ 스케일은 wrist 기준 병진에만 적용 (wrist 월드 위치는 그대로)
        self.declare_parameter('tf.world_absolute_scale.enable', True)
        self.declare_parameter('tf.world_absolute_scale.target_length', 0.06)   # meter (예: 60mm)
        self.declare_parameter('tf.world_absolute_scale.finger_name', 'index')  # 기준 손가락
        self.declare_parameter('tf.world_absolute_scale.joint_name', 'mcp')  # 기준 손가락
        self.declare_parameter('tf.world_absolute_scale.eps', 1e-6)
        self.declare_parameter('tf.world_absolute_scale.EMA_smooth_alpha', 0.3)     # EMA 알파(0=off)
        self.declare_parameter('tf.world_absolute_scale.suffix', 'world_abs')
        self.declare_parameter('tf.world_absolute_scale.max_scale_step', 0.0)   # 프레임당 s 변화 제한(0=off)

        # 파라미터 읽기
        self.hands_topic = self.get_parameter('hands_topic').get_parameter_value().string_value
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tf_norm_enable = bool(self.get_parameter('tf.norm.enable').value)
        self.tf_canon_enable = bool(self.get_parameter('tf.canonical.enable').value)
        self.tf_canon_norm_enable = bool(self.get_parameter('tf.canonical_norm.enable').value)
        self.tf_canon_norm_scale = float(self.get_parameter('tf.canonical_norm.scale').value)
        self.tf_world_abs_scale_enable = bool(self.get_parameter('tf.world_absolute_scale.enable').value)
        self.tf_world_abs_scale_target_length = float(self.get_parameter('tf.world_absolute_scale.target_length').value)
        self.tf_world_abs_scale_finger_name = str(self.get_parameter('tf.world_absolute_scale.finger_name').value).lower()
        self.tf_world_abs_scale_joint_name = str(self.get_parameter('tf.world_absolute_scale.joint_name').value).lower()
        self.tf_world_abs_scale_eps    = float(self.get_parameter('tf.world_absolute_scale.eps').value)
        self.tf_world_abs_scale_EMA_alpha  = float(self.get_parameter('tf.world_absolute_scale.EMA_smooth_alpha').value)
        self.tf_world_abs_scale_suffix = str(self.get_parameter('tf.world_absolute_scale.suffix').value)
        self.tf_world_abs_scale_max_step = float(self.get_parameter('tf.world_absolute_scale.max_scale_step').value)

        self.get_logger().info(f"self.hands_topic: {self.hands_topic}")
        self.get_logger().info(f"self.use_depth: {self.use_depth}")
        self.get_logger().info(f"self.depth_topic: {self.depth_topic}")
        self.get_logger().info(f"self.camera_info_topic: {self.camera_info_topic}")
        self.get_logger().info(f"self.camera_frame: {self.camera_frame}")
        self.get_logger().info(f"self.tf_norm_enable: {self.tf_norm_enable}")
        self.get_logger().info(f"self.tf_canon_enable: {self.tf_canon_enable}")
        self.get_logger().info(f"self.tf_canon_norm_enable: {self.tf_canon_norm_enable}")
        self.get_logger().info(f"self.tf_canon_norm_scale: {self.tf_canon_norm_scale}")
        self.get_logger().info(f"self.tf_world_abs_scale_enable: {self.tf_world_abs_scale_enable}")
        self.get_logger().info(f"self.tf_world_abs_scale_target_length: {self.tf_world_abs_scale_target_length}")
        self.get_logger().info(f"self.tf_world_abs_scale_finger_name: {self.tf_world_abs_scale_finger_name}")
        self.get_logger().info(f"self.tf_world_abs_scale_joint_name: {self.tf_world_abs_scale_joint_name}")
        self.get_logger().info(f"self.tf_world_abs_scale_eps: {self.tf_world_abs_scale_eps}")
        self.get_logger().info(f"self.tf_world_abs_scale_EMA_alpha: {self.tf_world_abs_scale_EMA_alpha}")
        self.get_logger().info(f"self.tf_world_abs_scale_suffix: {self.tf_world_abs_scale_suffix}")
        self.get_logger().info(f"self.tf_world_abs_scale_max_step: {self.tf_world_abs_scale_max_step}")

        # 파라미터 동적 업데이트 콜백
        self.add_on_set_parameters_callback(self.param_callback)

        # 브로드캐스터/브릿지
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # CameraInfo (기본값)
        self.camera_info = CameraInfo()
        self.camera_info.width = 1280
        self.camera_info.height = 720

        self.depth = None
        self.K = None  # fx, fy, cx, cy (필요 시 사용)

        # 구독
        self.sub_hands = self.create_subscription(Hands, self.hands_topic, self.cb_hands, 1)
        self.sub_info  = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 1)
        if self.use_depth:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)

        self.get_logger().info(f"[handpose_tf] subscribe: {self.hands_topic}, use_depth={self.use_depth}")

        # solver
        self._solvers = {
            "left": LandmarkToHandPose(flip_x=True, hand_label="left"),
            "right": LandmarkToHandPose(flip_x=True, hand_label="right")
        }

        # 손별 EMA 상태 (world_absolute_scale 전용)
        self._s_prev = {"left": None, "right": None}

        # ------ hand_config 로드 (캐시됨) ------
        self.cfg = get_config()
        self.finger_joint_map = build_finger_joint_map(self.cfg)
        # 사용 가능한 손가락 이름 (thumb/index/middle/ring/pinky …)
        self.available_fingers = set(self.cfg.get("fingers", {}).keys())

        # 유효 finger 이름 체크 (설정파일 기반)
        if self.tf_world_abs_scale_finger_name not in self.available_fingers:
            self.get_logger().warning(
                f"Unknown scale finger '{self.tf_world_abs_scale_finger_name}', fallback to 'index'"
            )
            self.tf_world_abs_scale_finger_name = "index"

    # ------------------------ Depth / CameraInfo ------------------------

    def cb_depth(self, msg: Image):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if d.dtype == np.uint16:
                self.depth = d.astype(np.float32) / 1000.0  # mm -> m
            else:
                self.depth = d.astype(np.float32)
        except Exception as e:
            self.get_logger().warning(f"depth conv error: {e}")

    def cb_info(self, msg: CameraInfo):
        self.camera_info = msg

    # ------------------------ TF Publish Helpers ------------------------
    def make_tf_from_matrix(
        self,
        parent:str,
        child:str,
        transform_matrix:np.ndarray,
        stamp)-> TransformStamped:
        """Publish TF coordinate system from {parent} to {child} using 4x4 {transform_matrix}.

        Args:
            parent (str): name of source coordinate system
            child (str): name of target coordinate system
            transform_matrix (np.ndarray): 4x4 homogeneous transfrom matrix
            stamp (builtin_interfaces.msg.Time): ROS2 time system

        Returns:
            TransformStamped: TF results
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        rotm = transform_matrix[:3,:3]
        q = rotm_to_quat(rotm)
        # self.get_logger().info(f"q:{q}")
        t.transform.translation.x = float(transform_matrix[0,3])
        t.transform.translation.y = float(transform_matrix[1,3])
        t.transform.translation.z = float(transform_matrix[2,3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def _send_tf_from_matrix(
        self,
        parent:str,
        child:str,
        transform_matrix:np.ndarray,
        stamp) -> bool:
        """Publish TF coordinate system from {parent} to {child} using 4x4 {transform_matrix}.

        Args:
            parent (str): name of source coordinate system
            child (str): name of target coordinate system
            transform_matrix (np.ndarray): 4x4 homogeneous transfrom matrix
            stamp (builtin_interfaces.msg.Time): ROS2 time system
        Returns:
            bool: success
        """
        t = self.make_tf_from_matrix(
            parent=parent,
            child=child,
            transform_matrix=transform_matrix,
            stamp=stamp
        )
        self.tf_broadcaster.sendTransform(t)
        return True

    def _publish_hand_tfs(self, label: str, frames, suffix: str, stamp, rel_scale: float = 1.0):
        """
        frames: LandmarkToHandPose.compute() 결과 (T_input2wrist, T_wrist2joint dict 등)
        suffix: "norm"/"canon"/"canon_norm"/custom
        rel_scale: wrist 기준으로 joint들의 병진(t)만 곱할 스케일 (회전/손목 위치는 그대로)
        """
        input_frame = self.camera_frame
        wrist_frame = f"hand_{label}_wrist_{suffix}"

        # input → wrist (그대로 퍼블리시: wrist의 월드 위치/자세는 유지)
        T_input2wrist = frames.T_input2wrist.copy()
        # 이미지 좌상단 원점 → 카메라 옵티컬 프레임 중심 정합(기존 코드 유지)
        T_input2wrist[0, 3] += 0.5 * (self.camera_info.width / self.camera_info.width)
        T_input2wrist[1, 3] -= 0.5 * (self.camera_info.height / self.camera_info.width)
        self._send_tf_from_matrix(parent=input_frame, child=wrist_frame,
                      transform_matrix=T_input2wrist, stamp=stamp)

        # wrist → joints (병진만 rel_scale 배)
        for (finger, jname), T in frames.T_wrist2joint.items():
            Ts = T.copy()
            Ts[:3, 3] *= float(rel_scale)  # wrist 기준 스케일 적용
            child = f"hand_{label}_{finger}_{jname}_{suffix}"
            self._send_tf_from_matrix(parent=wrist_frame, child=child, transform_matrix=Ts, stamp=stamp)

    # ------------------------ world_absolute_scale ------------------------

    def _compute_world_abs_scale_from_frames(self, label: str, frames) -> float:
        """
        canon_norm으로 만든 frames에서 wrist→MCP(finger) 변환의 병진 길이를 읽어
        target_length에 맞추는 상대 스케일 s(label)을 계산. EMA/스텝 클램프 포함.
        """
        key = (self.tf_world_abs_scale_finger_name, self.tf_world_abs_scale_joint_name)
        T = frames.T_wrist2joint.get(key)
        if T is None:
            # 느슨한 키 탐색(대소문자/동의어)
            for (f, j), Tv in frames.T_wrist2joint.items():
                if str(f).lower() == self.tf_world_abs_scale_finger_name and str(j).lower() in ("mcp", "mcp_joint"):
                    T = Tv
                    break
        if T is None:
            self.get_logger().warn(f"[{label}] wrist2joint missing MCP for finger '{self.tf_world_abs_scale_finger_name}'")
            prev = self._s_prev.get(label)
            return float(prev) if (prev is not None) else 1.0

        t = T[:3, 3]
        L_obs = float(np.linalg.norm(t))
        if not np.isfinite(L_obs) or L_obs <= 0.0:
            prev = self._s_prev.get(label)
            return float(prev) if (prev is not None) else 1.0

        eps = float(self.tf_world_abs_scale_eps)
        L_real = float(self.tf_world_abs_scale_target_length)
        s_raw = L_real / max(L_obs, eps)

        a = float(self.tf_world_abs_scale_EMA_alpha)
        prev = self._s_prev.get(label)
        if a > 0.0 and prev is not None:
            s_tmp = (1.0 - a) * prev + a * s_raw
        else:
            s_tmp = s_raw

        max_step = float(self.tf_world_abs_scale_max_step)
        if prev is not None and max_step > 0.0:
            delta = np.clip(s_tmp - prev, -max_step, max_step)
            s = prev + float(delta)
        else:
            s = s_tmp

        self._s_prev[label] = float(s)
        return float(s)

    # ------------------------ 메인 콜백 ------------------------

    def cb_hands(self, msg: Hands):
        """핸드 랜드마크 수신 시 TF 퍼블리시"""
        stamp = msg.header.stamp

        for hand in msg.hands:
            # 최대 2손(좌/우) 가정
            if hand.id > 2:
                self.get_logger().warn("More than 2 hands detected; ignoring extra hands.")
                continue

            if len(hand.landmarks_canon) != 63:
                continue
            lm_norm  = np.asarray(hand.landmarks_norm, dtype=np.float32).reshape(21, 3)
            lm_canon = np.asarray(hand.landmarks_canon, dtype=np.float32).reshape(21, 3)

            label = hand.label.lower()
            if label not in self._solvers:
                continue
            solver = self._solvers[label]

            # 1) norm 기반
            if self.tf_norm_enable:
                solver.update_landmarks(lm_norm)
                frames_norm = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_norm, suffix="norm", stamp=stamp, rel_scale=1.0)

            # 2) canonical 기반 (원본)
            if self.tf_canon_enable:
                solver.update_landmarks(lm_canon)
                frames_canon = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_canon, suffix="canon", stamp=stamp, rel_scale=1.0)

            # 3) canonical_norm 기반 (base scale만 적용)
            frames_canon_norm = None
            if self.tf_canon_norm_enable:
                lm_canon_scaled = lm_canon * self.tf_canon_norm_scale
                solver.update_landmarks(lm_canon_scaled)
                frames_canon_norm = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_canon_norm, suffix="canon_norm", stamp=stamp, rel_scale=1.0)

            # 4) world_absolute_scale (양손 독립 절대 스케일; wrist 기준 병진만 스케일)
            # world_absolute_scale must have canonical_norm frame.
            if self.tf_world_abs_scale_enable:
                if self.tf_canon_norm_enable:
                    s_rel = self._compute_world_abs_scale_from_frames(label, frames_canon_norm)
                    self._publish_hand_tfs(label, frames_canon_norm, suffix=self.tf_world_abs_scale_suffix, stamp=stamp, rel_scale=s_rel)
                else:
                    # if you turn off the calculation canonical_norm frame.
                    # you must calculate the canonical_norm frame for world_abs_frame
                    lm_canon_scaled = lm_canon * self.tf_canon_norm_scale
                    solver.update_landmarks(lm_canon_scaled)
                    frames_canon_norm = solver.compute(label=label)
                    s_rel = self._compute_world_abs_scale_from_frames(label, frames_canon_norm)
                    self._publish_hand_tfs(label, frames_canon_norm, suffix=self.tf_world_abs_scale_suffix, stamp=stamp, rel_scale=s_rel)

    # ------------------------ 파라미터 업데이트 ------------------------

    def param_callback(self, params):
        for param in params:
            if param.name == 'tf.norm.enable':
                self.tf_norm_enable = bool(param.value)
            elif param.name == 'tf.canonical.enable':
                self.tf_canon_enable = bool(param.value)
            elif param.name == 'tf.canonical_norm.enable':
                self.tf_canon_norm_enable = bool(param.value)
            elif param.name == 'tf.canonical_norm.scale':
                self.tf_canon_norm_scale = float(param.value)
            elif param.name == 'tf.world_absolute_scale.enable':
                self.tf_world_abs_scale_enable = bool(param.value)
            elif param.name == 'tf.world_absolute_scale.target_length':
                self.tf_world_abs_scale_target_length = float(param.value)
            elif param.name == 'tf.world_absolute_scale.finger_name':
                name = str(param.value).lower()
                # 설정파일 기반 검증
                if name in self.available_fingers:
                    self.tf_world_abs_scale_finger_name = name
                else:
                    self.get_logger().warning(f"Unknown scale finger '{name}', keep '{self.tf_world_abs_scale_finger_name}'")
            elif param.name == 'tf.world_absolute_scale.norm_eps':
                self.tf_world_abs_scale_eps = float(param.value)
            elif param.name == 'tf.world_absolute_scale.smooth_alpha':
                self.tf_world_abs_scale_EMA_alpha = float(param.value)
            elif param.name == 'tf.world_absolute_scale.suffix':
                self.tf_world_abs_scale_suffix = str(param.value)
            elif param.name == 'tf.world_absolute_scale.max_scale_step':
                self.tf_world_abs_scale_max_step = float(param.value)

            self.get_logger().info(f"Parameter '{param.name}' updated.")

        return SetParametersResult(successful=True)

# ------------------------ 엔트리 ------------------------

def main():
    rclpy.init()
    node = HandPoseTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
