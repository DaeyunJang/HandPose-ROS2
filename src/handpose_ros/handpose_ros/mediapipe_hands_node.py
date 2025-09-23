import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np
import struct

from handpose_interfaces.msg import Hands, HandLandmarks

# mediapipe
import mediapipe as mp
try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

class MediaPipeHandsNode(Node):
    """
    ROS 2 node that runs MediaPipe Hands on an incoming Image stream, publishes:
      - a custom Hands message (with normalized/canonical/world landmarks),
      - optionally PointCloud2 for left/right hands and the combined point cloud.

    Handedness labels ('left'/'right') from MediaPipe are used directly to route
    per-hand point clouds to /hands/points/hand_left and /hands/points/hand_right.
    The combined cloud is published to /hands/points when enabled.
    """
    def __init__(self):
        """Initialize publishers, subscriber, parameters, and MediaPipe runtime."""
        super().__init__('mediapipe_hands_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('max_num_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.95)
        self.declare_parameter('min_tracking_confidence', 0.95)
        self.declare_parameter('draw', False)           # overlay for debugging
        self.declare_parameter('flip_image', True)      # horizontal flip input
        self.declare_parameter('use_pointcloud', True)  # enable PointCloud2 output

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.max_hands = self.get_parameter('max_num_hands').get_parameter_value().integer_value
        self.det_conf = float(self.get_parameter('min_detection_confidence').value)
        self.trk_conf = float(self.get_parameter('min_tracking_confidence').value)
        self.draw = bool(self.get_parameter('draw').value)
        self.flip = bool(self.get_parameter('flip_image').value)
        self.use_pointcloud = bool(self.get_parameter('use_pointcloud').value)

        # custom interfaces
        self.publisher_topic_name = "hands/detections"
        self.pub = self.create_publisher(Hands, self.publisher_topic_name, 1)

        # PointCloud2 publishers (publishing is gated by "use_pointcloud")
        self.pc_pub_all = self.create_publisher(PointCloud2, 'hands/points', 1)  # hand_all
        self.pc_pub_left  = self.create_publisher(PointCloud2, 'hands/points/hand_left', 1)
        self.pc_pub_right = self.create_publisher(PointCloud2, 'hands/points/hand_right', 1)

        # Image subscriber
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, 1)

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_hands,
            min_detection_confidence=self.det_conf,
            min_tracking_confidence=self.trk_conf
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles
        self.pub_mp_overlay_image = self.create_publisher(Image, 'mp_overlay_image', 1)
        
        # Logs
        self.get_logger().info(f"[mediapipe] subscribe image_topic: {self.image_topic}")
        self.get_logger().info(f"[mediapipe] max_num_hands: {self.max_hands}")
        self.get_logger().info(f"[mediapipe] min_detection_confidence: {self.det_conf}")
        self.get_logger().info(f"[mediapipe] min_tracking_confidence: {self.trk_conf}")
        self.get_logger().info(f"[mediapipe] draw: {self.draw}")
        self.get_logger().info(f"[mediapipe] flip_image: {self.flip}")
        self.get_logger().info(f"[mediapipe] use_pointcloud: {self.use_pointcloud}")
        self.get_logger().info(f"[mediapipe] publisher topic name: {self.publisher_topic_name}")
        self.get_logger().info(f"[mediapipe] MediaPipiHandsNode is created.")

    @staticmethod
    def _norm_to_canonical(lm_norm: np.ndarray, w: int, h: int) -> np.ndarray:
        """
        Convert normalized landmarks ([0..1] in x,y and normalized z) to pixel space.

        Args:
            lm_norm: (21,3) array with normalized (x,y,z) landmarks.
            w: image width in pixels.
            h: image height in pixels.

        Returns:
            (21,3) array in pixel units (z scaled by image width).
        """
        lm = lm_norm.copy()
        lm[:,0] = lm[:,0] * float(w)
        lm[:,1] = lm[:,1] * float(h)
        lm[:,2] = lm[:,2] * float(w)   # z는 W 기준 스케일
        return lm

    def _pack_rgb(self, r, g, b):
        """
        Pack 8-bit RGB into a single float32 for PCL-compatible PointCloud2 'rgb' field.

        Args:
            r, g, b: integers in [0, 255]

        Returns:
            float32 representing packed RGB.
        """
        return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

    def _np_to_pointcloud2(self, pts_xyz: np.ndarray, header: Header, rgb=None) -> PointCloud2:
        """
        Convert Nx3 numpy array (and optional per-point packed rgb) to PointCloud2.

        Args:
            pts_xyz: (N,3) float32 positions.
            header: std_msgs/Header to stamp the message.
            rgb: None or (N,) float32 packed rgb values.

        Returns:
            sensor_msgs/PointCloud2 message.
        """
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = int(pts_xyz.shape[0])
        msg.is_bigendian = False
        msg.is_dense = True

        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12
        if rgb is not None:
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
            point_step = 16

        msg.fields = fields
        msg.point_step = point_step
        msg.row_step = msg.point_step * msg.width

        if msg.width == 0:
            msg.data = b''
            return msg

        if rgb is None:
            buf = pts_xyz.astype(np.float32).tobytes()
        else:
            arr = np.concatenate([pts_xyz.astype(np.float32), rgb.reshape(-1,1).astype(np.float32)], axis=1)
            buf = arr.tobytes()
        msg.data = buf
        return msg

    def cb_image(self, msg: Image):
        """
        Image callback:
          1) Convert ROS Image to BGR (cv_bridge).
          2) Optional horizontal flip for selfie-view alignment.
          3) Run MediaPipe Hands on RGB.
          4) Build and publish normalized, canonical points using custom Hands interfaces.
          5) Optionally build and publish PointCloud2 for left/right/all.
          6) Optionally publish an overlay image for debugging.
        """
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if bgr is None:
            return
        
        # Optional horizontal flip at input to match expected handedness in camera view
        if self.flip:
            if _HAS_CV2:
                bgr = cv2.flip(bgr, 1)     # 1 = horizontal
            else:
                bgr = bgr[:, ::-1]         # fallback without cv2
                
        h, w = bgr.shape[:2]
        image = bgr[..., ::-1].copy()      # BGR -> RGB
        annotated_image = image.copy()
        res = self.hands.process(image)

        out = Hands()
        out.header = msg.header
        
        # Buffers for combined point cloud (only used when use_pointcloud is True)
        all_pts = []
        all_rgb = []

        if not res.multi_hand_landmarks:
            # Nothing detected, publish empty Hands (and overlay if requested)
            self.pub.publish(out)
            
            annotated_image = cv2.flip(annotated_image, 1)  # re flip
            overlay_bgr = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
            self.pub_mp_overlay_image.publish(self.bridge.cv2_to_imgmsg(overlay_bgr))   # convert ROS message type
            return

        # Handedness
        # Extract handedness ('left'/'right' + score/index) for each hand
        handed_list = []
        if res.multi_handedness:
            for hnd in res.multi_handedness:
                cat = hnd.classification[0]
                handed_list.append((cat.label.lower(), float(cat.score), int(cat.index)))
        else:
            handed_list = [("right", 1.0, -1)] * len(res.multi_hand_landmarks)

        for i, lms in enumerate(res.multi_hand_landmarks):
            # Normalized landmarks
            lm_norm = np.array([[p.x, p.y, p.z] for p in lms.landmark], dtype=np.float32)
            
            # Canonical landmarks (pixel)
            lm_canon = self._norm_to_canonical(lm_norm, w, h)

            # Prefer "world" landmarks if available (MediaPipe's world space ~meters)
            if res.multi_hand_world_landmarks and i < len(res.multi_hand_world_landmarks):
                lm_world = np.array([[p.x, p.y, p.z] for p in res.multi_hand_world_landmarks[i].landmark],
                                    dtype=np.float32)
                pts = lm_world.copy()   # fallback: pixel-based 3D approximation
            else:
                lm_world = np.array([], dtype=np.float32)
                pts = lm_canon.copy()  # ★ fallback 보장

            label, score, idx = handed_list[i] if i < len(handed_list) else ("unknown", 0.0, -1)

            # Color by handedness (fixed and stable across frames)
            if label == "left":
                r, g, b = (0, 255, 0)      # left = green
            elif label == "right":
                r, g, b = (255, 0, 0)      # right = red
            else:
                r, g, b = (255, 255, 0)    # unknown = yellow
            rgb = np.array([self._pack_rgb(r,g,b)] * pts.shape[0], dtype=np.float32)
            
            # Fill custom message per hand
            m = HandLandmarks()
            m.header = msg.header
            m.id = i
            m.label = label
            m.score = float(score)
            m.handed_index = int(idx)
            m.width = w
            m.height = h
            m.landmarks_norm = lm_norm.reshape(-1).tolist()
            m.landmarks_canon = lm_canon.reshape(-1).tolist()
            m.landmarks_world = lm_world.reshape(-1).tolist() if lm_world.size else []
            out.hands.append(m)
            
            # image overlay
            self.mp_drawing.draw_landmarks(
                image=annotated_image,
                landmark_list=lms,
                connections=self.mp_hands.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_styles.get_default_hand_connections_style()
            )

            # Per-hand PointCloud2 publish using handedness routing
            if self.use_pointcloud:
                pc_hand = self._np_to_pointcloud2(pts, msg.header, rgb)
                if label == "left":
                    self.pc_pub_left.publish(pc_hand)
                elif label == "right":
                    self.pc_pub_right.publish(pc_hand)
                # unknown이면 개별 토픽 생략(원하면 all만 포함)

                # Accumulate for combined cloud
                all_pts.append(pts)
                all_rgb.append(rgb)

        # Combined PointCloud2
        if self.use_pointcloud and all_pts:
            all_pts_np = np.concatenate(all_pts, axis=0)
            all_rgb_np = np.concatenate(all_rgb, axis=0)
            self.pc_pub_all.publish(self._np_to_pointcloud2(all_pts_np, msg.header, all_rgb_np))

        # Publish custom Hands message
        self.pub.publish(out)
        
        # Publish overlay image for visualization (mirrored back for display)
        annotated_image = cv2.flip(annotated_image, 1)  # re flip
        overlay_bgr = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
        self.pub_mp_overlay_image.publish(self.bridge.cv2_to_imgmsg(overlay_bgr))   # convert ROS message type

def main():
    """ROS 2 entrypoint."""

    rclpy.init()
    node = MediaPipeHandsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
