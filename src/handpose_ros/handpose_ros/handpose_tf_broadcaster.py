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
from handpose_ros.algebra_utils import *  # using rotm_to_quat, etc.
from handpose_ros.hand_config_loader import get_config, build_finger_joint_map


class HandPoseTFNode(Node):
    """
    Broadcasts TF frames for hand landmarks derived from a custom `Hands` message.
    Supports:
      - raw normalized frames,
      - canonical frames,
      - canonical frames with a base normalization scale (canonical_norm),
      - world-absolute scaling per hand: scales only the translation from wrist
        to joints to match a target physical length (per hand), with EMA smoothing
        and optional per-frame scale change clamping.

    The node optionally subscribes to depth and camera info, and uses a configurable
    camera frame as the TF parent for wrist frames.
    """

    def __init__(self):
        """Initialize parameters, subscriptions, TF broadcaster, solvers, and caches."""
        super().__init__('handpose_tf_broadcaster')

        # ------------------------ Parameters ------------------------
        self.declare_parameter('hands_topic', 'hands/detections')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')    # TF name

        # Which TF variants to publish
        self.declare_parameter('tf.norm.enable', True)
        self.declare_parameter('tf.canonical.enable', True)

        # canon_norm: apply only a base scale (no auto scaling)
        self.declare_parameter('tf.canonical_norm.enable', True)
        self.declare_parameter('tf.canonical_norm.scale', 1/1280)  # base scale

        # real world_absolute_scale (per-hand absolute wrist→MCP scale)
        # ※ The scale is applied to translations w.r.t. wrist only (wrist world pose remains unchanged)
        self.declare_parameter('tf.world_absolute_scale.enable', True)
        self.declare_parameter('tf.world_absolute_scale.target_length', 0.06)   # meters (e.g., 60 mm)
        self.declare_parameter('tf.world_absolute_scale.finger_name', 'index')  # reference finger
        self.declare_parameter('tf.world_absolute_scale.joint_name', 'mcp')     # reference joint
        self.declare_parameter('tf.world_absolute_scale.eps', 1e-6)
        self.declare_parameter('tf.world_absolute_scale.EMA_smooth_alpha', 0.3) # EMA alpha (0=off)
        self.declare_parameter('tf.world_absolute_scale.suffix', 'world_abs')
        self.declare_parameter('tf.world_absolute_scale.max_scale_step', 0.0)   # per-frame change clamp (0=off)

        # Read parameters
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

        # Dynamic parameter update callback
        self.add_on_set_parameters_callback(self.param_callback)

        # Broadcaster / bridge
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # Default CameraInfo
        self.camera_info = CameraInfo()
        self.camera_info.width = 1280
        self.camera_info.height = 720

        self.depth = None
        self.K = None  # fx, fy, cx, cy (use if needed)

        # Subscriptions
        self.sub_hands = self.create_subscription(Hands, self.hands_topic, self.cb_hands, 1)
        self.sub_info  = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 1)
        if self.use_depth:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)

        self.get_logger().info(f"[handpose_tf] subscribe: {self.hands_topic}, use_depth={self.use_depth}")

        # Solvers per hand (left/right)
        self._solvers = {
            "left": LandmarkToHandPose(flip_x=True, hand_label="left"),
            "right": LandmarkToHandPose(flip_x=True, hand_label="right")
        }

        # Per-hand EMA state (for world_absolute_scale)
        self._s_prev = {"left": None, "right": None}

        # ------ Load hand_config (cached) ------
        self.cfg = get_config()
        self.finger_joint_map = build_finger_joint_map(self.cfg)
        # Available finger names (thumb/index/middle/ring/pinky …)
        self.available_fingers = set(self.cfg.get("fingers", {}).keys())

        # Validate reference finger name (based on config)
        if self.tf_world_abs_scale_finger_name not in self.available_fingers:
            self.get_logger().warning(
                f"Unknown scale finger '{self.tf_world_abs_scale_finger_name}', fallback to 'index'"
            )
            self.tf_world_abs_scale_finger_name = "index"

    # ------------------------ Depth / CameraInfo ------------------------

    def cb_depth(self, msg: Image):
        """
        Depth image callback.
        Converts incoming depth to float32 in meters and caches it.
        """
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if d.dtype == np.uint16:
                self.depth = d.astype(np.float32) / 1000.0  # mm -> m
            else:
                self.depth = d.astype(np.float32)
        except Exception as e:
            self.get_logger().warning(f"depth conv error: {e}")

    def cb_info(self, msg: CameraInfo):
        """
        CameraInfo callback.
        Stores the latest camera intrinsics and dimensions.
        """
        self.camera_info = msg

    # ------------------------ TF Publish Helpers ------------------------
    def make_tf_from_matrix(
        self,
        parent: str,
        child: str,
        transform_matrix: np.ndarray,
        stamp) -> TransformStamped:
        """
        Create a TransformStamped from a 4x4 homogeneous transform matrix.

        Args:
            parent (str): parent frame id
            child (str): child frame id
            transform_matrix (np.ndarray): (4,4) homogeneous transform
            stamp (builtin_interfaces.msg.Time): ROS2 timestamp

        Returns:
            TransformStamped: transform ready to be broadcast
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        rotm = transform_matrix[:3, :3]
        q = rotm_to_quat(rotm)
        t.transform.translation.x = float(transform_matrix[0, 3])
        t.transform.translation.y = float(transform_matrix[1, 3])
        t.transform.translation.z = float(transform_matrix[2, 3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        return t

    def _send_tf_from_matrix(
        self,
        parent: str,
        child: str,
        transform_matrix: np.ndarray,
        stamp) -> bool:
        """
        Broadcast a TF transform from a 4x4 matrix.

        Args:
            parent (str): parent frame id
            child (str): child frame id
            transform_matrix (np.ndarray): (4,4) homogeneous transform
            stamp (builtin_interfaces.msg.Time): ROS2 timestamp

        Returns:
            bool: True if sent
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
        Publish a wrist frame and all joint frames for a given hand label.

        Args:
            label (str): 'left' or 'right'
            frames: output of LandmarkToHandPose.compute(), including:
                - T_input2wrist (4x4)
                - T_wrist2joint dict mapping (finger, joint) -> 4x4
            suffix (str): e.g., "norm", "canon", "canon_norm", or a custom suffix
            stamp: ROS2 timestamp
            rel_scale (float): scale applied to translations (from wrist to joints) only

        Behavior:
            - Parent is `self.camera_frame`.
            - First publishes input -> wrist (no scaling; wrist world pose is kept).
            - Then publishes wrist -> each joint, scaling translation component only by rel_scale.
        """
        input_frame = self.camera_frame
        wrist_frame = f"hand_{label}_wrist_{suffix}"

        # input → wrist (as-is; wrist world pose preserved)
        T_input2wrist = frames.T_input2wrist.copy()
        # Adjust image-origin (top-left) to camera optical center (keep previous alignment behavior)
        T_input2wrist[0, 3] += 0.5 * (self.camera_info.width / self.camera_info.width)
        T_input2wrist[1, 3] -= 0.5 * (self.camera_info.height / self.camera_info.width)
        self._send_tf_from_matrix(parent=input_frame, child=wrist_frame,
                                  transform_matrix=T_input2wrist, stamp=stamp)

        # wrist → joints (scale translation only)
        for (finger, jname), T in frames.T_wrist2joint.items():
            Ts = T.copy()
            Ts[:3, 3] *= float(rel_scale)  # apply scale on wrist-relative translation only
            child = f"hand_{label}_{finger}_{jname}_{suffix}"
            self._send_tf_from_matrix(parent=wrist_frame, child=child, transform_matrix=Ts, stamp=stamp)

    # ------------------------ world_absolute_scale ------------------------

    def _compute_world_abs_scale_from_frames(self, label: str, frames) -> float:
        """
        Compute a relative scale s(label) so that the wrist→MCP(finger) translation
        matches the configured target_length. Uses EMA smoothing and optional per-frame
        change clamping.

        Args:
            label (str): 'left' or 'right'
            frames: canonical_norm frames for that hand (required for stable length)

        Returns:
            float: scale factor to apply to wrist→joint translations.
        """
        key = (self.tf_world_abs_scale_finger_name, self.tf_world_abs_scale_joint_name)
        T = frames.T_wrist2joint.get(key)
        if T is None:
            # Loose key search (case-insensitive, synonyms)
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

    # ------------------------ Main callback ------------------------

    def cb_hands(self, msg: Hands):
        """
        Main Hands message callback:
          - For each detected hand, compute frames for:
            1) normalized landmarks (if enabled),
            2) canonical landmarks (if enabled),
            3) canonical_norm (base-scaled canonical, if enabled),
            4) world_abs (absolute-scale canonical_norm with per-hand scale from wrist→MCP).
          - Publish the corresponding TF trees for the specified suffixes.
        """
        stamp = msg.header.stamp

        for hand in msg.hands:
            # Assume at most two hands (left/right)
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

            # 1) norm-based
            if self.tf_norm_enable:
                solver.update_landmarks(lm_norm)
                frames_norm = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_norm, suffix="norm", stamp=stamp, rel_scale=1.0)

            # 2) canonical-based (original)
            if self.tf_canon_enable:
                solver.update_landmarks(lm_canon)
                frames_canon = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_canon, suffix="canon", stamp=stamp, rel_scale=1.0)

            # 3) canonical_norm (apply only base scale)
            frames_canon_norm = None
            if self.tf_canon_norm_enable:
                lm_canon_scaled = lm_canon * self.tf_canon_norm_scale
                solver.update_landmarks(lm_canon_scaled)
                frames_canon_norm = solver.compute(label=label)
                self._publish_hand_tfs(label, frames_canon_norm, suffix="canon_norm", stamp=stamp, rel_scale=1.0)

            # 4) world_absolute_scale (per-hand absolute scale; scale only wrist-relative translations)
            # world_absolute_scale must have canonical_norm frame.
            if self.tf_world_abs_scale_enable:
                if self.tf_canon_norm_enable:
                    s_rel = self._compute_world_abs_scale_from_frames(label, frames_canon_norm)
                    self._publish_hand_tfs(label, frames_canon_norm, suffix=self.tf_world_abs_scale_suffix, stamp=stamp, rel_scale=s_rel)
                else:
                    # If canonical_norm computation is turned off,
                    # compute canonical_norm here for world_abs purposes.
                    lm_canon_scaled = lm_canon * self.tf_canon_norm_scale
                    solver.update_landmarks(lm_canon_scaled)
                    frames_canon_norm = solver.compute(label=label)
                    s_rel = self._compute_world_abs_scale_from_frames(label, frames_canon_norm)
                    self._publish_hand_tfs(label, frames_canon_norm, suffix=self.tf_world_abs_scale_suffix, stamp=stamp, rel_scale=s_rel)

    # ------------------------ Parameter updates ------------------------

    def param_callback(self, params):
        """
        Dynamic parameter update callback. Applies toggles and numeric updates
        for TF publication modes and scaling parameters. Validates finger name
        against the loaded configuration when necessary.
        """
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
                # Validate against config
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


# ------------------------ Entry point ------------------------

def main():
    """ROS 2 entry point."""
    rclpy.init()
    node = HandPoseTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
