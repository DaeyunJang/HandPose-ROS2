import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
from rcl_interfaces.msg import SetParametersResult

from handpose_interfaces.msg import Hands
from handpose_ros.landmark_to_handpose import LandmarkToHandPose
# from landmark_to_handpose import LandmarkToHandPose

class HandPoseTFNode(Node):
    def __init__(self):
        super().__init__('handpose_tf_broadcaster')

        # 파라미터
        self.declare_parameter('hands_topic', 'hands/detections')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_frame')    # For TF name
        self.declare_parameter('tf.norm.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.canonical.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.canonical.scale', 1/1280)

        self.hands_topic = self.get_parameter('hands_topic').get_parameter_value().string_value
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tf_norm_enable = self.get_parameter('tf.norm.enable').value
        self.tf_canon_enable = self.get_parameter('tf.canonical.enable').value
        self.tf_canon_scale = self.get_parameter('tf.canonical.scale').value
        
        # callback function for update parameters
        self.add_on_set_parameters_callback(self.param_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.camera_info = CameraInfo()
        self.camera_info.width = 1280
        self.camera_info.height = 720
        
        self.depth = None
        self.K = None  # fx, fy, cx, cy

        self.sub_hands = self.create_subscription(Hands, self.hands_topic, self.cb_hands, 1)
        self.sub_info  = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 1)

        if self.use_depth:
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.cb_depth, 10)

        self.get_logger().info(f"[handpose_tf] subscribe: {self.hands_topic}, use_depth={self.use_depth}")
        
        self._solvers = {
            "left": LandmarkToHandPose(flip_x=True, hand_label="left"),
            "right": LandmarkToHandPose(flip_x=True, hand_label="right")
        }

    def cb_depth(self, msg: Image):
        try:
            # RealSense ROS2: depth는 uint16 (mm). 미터 변환.
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if d.dtype == np.uint16:
                self.depth = d.astype(np.float32) / 1000.0
            else:
                # 이미 float32(m) 형태인 경우도 있음
                self.depth = d.astype(np.float32)
        except Exception as e:
            self.get_logger().warn(f"depth conv error: {e}")

    def cb_info(self, msg: CameraInfo):
        self.camera_info = msg
        
        # if msg.k and len(msg.k) == 9:
        #     fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        #     self.K = (float(fx), float(fy), float(cx), float(cy))

    def _depth_at(self, u, v, ksize=7):
        if self.depth is None:
            return np.nan
        H, W = self.depth.shape
        u = int(round(u)); v = int(round(v))
        x0 = max(0, u-ksize//2); x1 = min(W, u+ksize//2+1)
        y0 = max(0, v-ksize//2); y1 = min(H, v+ksize//2+1)
        patch = self.depth[y0:y1, x0:x1]
        vals = patch[np.isfinite(patch) & (patch > 0)]
        return float(np.median(vals)) if vals.size else float('nan')

    @staticmethod
    def _backproject(u, v, Z, fx, fy, cx, cy):
        X = (u - cx) / fx * Z
        Y = (v - cy) / fy * Z
        return np.array([X, Y, Z], dtype=np.float32)

    def _send_tf(self, parent, child, transform_matrix, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        # 행렬 → quaternion + translation
        R = transform_matrix[:3,:3]
        q = self._rotm_to_quat(R)
        t.transform.translation.x = float(transform_matrix[0,3])
        t.transform.translation.y = float(transform_matrix[1,3])
        t.transform.translation.z = float(transform_matrix[2,3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotm_to_quat(R):
        # 안전한 회전행렬 → 쿼터니언
        qw = np.sqrt(max(0, 1 + R[0,0] + R[1,1] + R[2,2])) / 2
        qx = (R[2,1] - R[1,2]) / (4*qw + 1e-9)
        qy = (R[0,2] - R[2,0]) / (4*qw + 1e-9)
        qz = (R[1,0] - R[0,1]) / (4*qw + 1e-9)
        return (qx,qy,qz,qw)

    def cb_hands(self, msg: Hands):
        """Callback funtion when subscriber reads image topic

        Args:
            msg (Hands): 'handpose_interfaces' package
        """
        stamp = msg.header.stamp

        # 👇 여기 안에 중첩 함수 정의
        def _publish_hand_tfs(label: str, frames, lm_norm, suffix: str):
            input_frame = self.camera_frame
            wrist_frame = f"hand_{label}_wrist_{suffix}"

            # center of normalized frame w.r.t name of 'camera_frame' -> in my case: camera_color_frame (Sometimes camera_color_optical_frame)
            T_cam2center = np.eye(4, dtype=np.float32)
            T_cam2center[0, 3] = 0.5 * (self.camera_info.width / self.camera_info.width)
            T_cam2center[1, 3] = 0.5 * (self.camera_info.height / self.camera_info.width)
            T_cam2center[2, 3] = 0
            self._send_tf(parent=input_frame, child=f'normalized_{input_frame}_center',
                          transform_matrix=T_cam2center, stamp=stamp)
            
            # input → wrist
            self._send_tf(parent=input_frame, child=wrist_frame,
                        transform_matrix=frames.T_input2wrist, stamp=stamp)

            # wrist → joints
            for (finger, jname), T in frames.T_wrist2joint.items():
                child = f"hand_{label}_{finger}_{jname}_{suffix}"
                self._send_tf(parent=wrist_frame, child=child, transform_matrix=T, stamp=stamp)

            # (옵션) depth anchoring → norm 좌표계 wrist 픽셀 기준
            if self.use_depth and (self.depth is not None) and (self.K is not None):
                u, v = float(lm_norm[0, 0]), float(lm_norm[0, 1])
                Z = self._depth_at(u, v, ksize=9)
                if np.isfinite(Z) and Z > 0:
                    fx, fy, cx, cy = self.K
                    p_cam = self._backproject(u, v, Z, fx, fy, cx, cy)
                    T_cam2wrist = np.eye(4, dtype=np.float32)
                    T_cam2wrist[:3, 3] = p_cam
                    self._send_tf(parent=self.camera_frame,
                                child=wrist_frame,
                                transform_matrix=T_cam2wrist,
                                stamp=stamp)
                    
        for hand in msg.hands:
            """Left and Right hand only
            and make sure that just less than 2 hands is used.
            """
            # Skip over 3rd hand.
            if hand.id > 2:
                self.get_logger().warn(f"Make sure that no more than 3 hands are included in the image.")
                # continue
                return
            
            # landmarks_canon → (21,3)
            if len(hand.landmarks_canon) != 63:
                continue
            lm_norm = np.asarray(hand.landmarks_norm, dtype=np.float32).reshape(21,3)
            lm_canon = np.asarray(hand.landmarks_canon, dtype=np.float32).reshape(21,3)
            
            label = hand.label.lower()
            solver = self._solvers[label]   # left or right solver 참조

            # norm 기반
            if self.tf_norm_enable:
                solver.update_landmarks(lm_norm)
                frames_norm = solver.compute(label=label)
                _publish_hand_tfs(label, frames_norm, lm_canon, suffix="norm")

            # canonical 기반 - 정규화는 해야함.
            if self.tf_canon_enable:
                solver.update_landmarks(lm_canon * self.tf_canon_scale)
                frames_canon = solver.compute(label=label)
                _publish_hand_tfs(label, frames_canon, lm_canon, suffix="canon")
                
    def param_callback(self, params):
        for param in params:
            if param.name == 'tf.norm.enable':
                self.tf_norm_enable = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_norm_enable}")
            elif param.name == 'tf.canonical.enable':
                self.tf_canon_enable = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_canon_enable}")
            elif param.name == 'tf.canonical.scale':
                self.tf_canon_scale = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_canon_scale}")

        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = HandPoseTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
