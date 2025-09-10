import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import SetParametersResult

from handpose_interfaces.msg import Hands
from handpose_ros.landmark_to_handpose import LandmarkToHandPose
from handpose_ros.algebra_utils import *
# from landmark_to_handpose import LandmarkToHandPose

class HandPoseTFNode(Node):
    def __init__(self):
        super().__init__('handpose_tf_broadcaster')

        # 파라미터
        self.declare_parameter('hands_topic', 'hands/detections')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')    # For TF name
        self.declare_parameter('tf.norm.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.canonical.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.canonical_norm.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.canonical_norm.scale', 1/1280)
        self.declare_parameter('tf.user_defined.enable', True)    # For TF topic On/Off
        self.declare_parameter('tf.user_defined.scale', 0.06/0.5)
        
        self.declare_parameter('tf.user_defined.suffix', 'gripper_scale')

        self.hands_topic = self.get_parameter('hands_topic').get_parameter_value().string_value
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.tf_norm_enable = self.get_parameter('tf.norm.enable').value
        self.tf_canon_enable = self.get_parameter('tf.canonical.enable').value
        self.tf_canon_norm_enable = self.get_parameter('tf.canonical_norm.enable').value
        self.tf_canon_norm_scale = self.get_parameter('tf.canonical_norm.scale').value
        self.tf_user_defined_enable = self.get_parameter('tf.user_defined.enable').value
        self.tf_user_defined_scale = self.get_parameter('tf.user_defined.scale').value
        self.tf_user_defined_suffix = self.get_parameter('tf.user_defined.suffix').get_parameter_value().string_value
        
        # callback function for update parameters
        self.add_on_set_parameters_callback(self.param_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

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

    def _send_tf(self, parent:str, child:str, transform_matrix:np.ndarray, stamp:Header):
        """Publish TF coordinate system from {parent} to {child} using 4x4 {transform_matrix}.

        Args:
            parent (str): name of source coordinate system
            child (str): name of target coordinate system
            transform_matrix (np.ndarray): 4x4 homogeneous transfrom matrix
            stamp (msg.header.stamp): ROS2 time system
        """
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        # 행렬 → quaternion + translation
        rotm = transform_matrix[:3,:3]
        
        q = rotm_to_quat(R=rotm)
        # self.get_logger().info(f"q:{q}")
        # q_R = R.from_matrix(rotm).as_quat()  # x,y,z,w
        # self.get_logger().info(f"q_R:{q_R}")
        # q2 = rotm_to_quat_v2(rotm)
        # self.get_logger().info(f"q2:{q2}")
        
        t.transform.translation.x = float(transform_matrix[0,3])
        t.transform.translation.y = float(transform_matrix[1,3])
        t.transform.translation.z = float(transform_matrix[2,3])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def cb_hands(self, msg: Hands):
        """Callback funtion when subscriber reads image topic

        Args:
            msg (Hands): 'handpose_interfaces' package
        """
        stamp = msg.header.stamp

        # 👇 여기 안에 중첩 함수 정의
        def _publish_hand_tfs(label: str, frames, suffix: str):
            input_frame = self.camera_frame
            wrist_frame = f"hand_{label}_wrist_{suffix}"
            
            # DY
            # Center of normalized frame w.r.t name of 'camera_frame' 
            #   -> in my case: camera_color_frame (Sometimes camera_color_optical_frame)
            # T_cam2center = np.eye(4, dtype=np.float32)
            # T_cam2center[0, 3] = 0.5 * (self.camera_info.width / self.camera_info.width)
            # T_cam2center[1, 3] = 0.5 * (self.camera_info.height / self.camera_info.width)
            # T_cam2center[2, 3] = 0
            # self._send_tf(parent=input_frame, child=f'normalized_{input_frame}_center',
            #               transform_matrix=T_cam2center, stamp=stamp)
            
            #################################################################################
            # input → wrist
            # 카메라 이미지는 x, y축이 0부터시작해서 해상도만큼 까지라 손목좌표계가 0~1 사이에 있음
            # 실제 realsense의 경우, camera_color_optical_frame의 위치와 맞춰주기 위해서 width와 height의 절반으로 원점을 일치시켜줌
            # 카메라 이미지는 좌측상단이 원점이고 우측으로 x, 아래로 y임
            ####### 중요 #######
            # 일단 기준 프레임은 camera_color_optical_frame으로 계산함.
            # 그리고 camera_color_frame을 기준으로 보면 ROS2 시스템에서 정의한 카메라좌표계기준이 되기는 함.
            # In ros2 TF system, coordinate system was transformed by Roty(-90)*Rotx(90) or same as 'camera_color_frame'
            # https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#ros2robot-vs-opticalcamera-coordination-systems
            T_input2wrist = frames.T_input2wrist.copy()
            # T_input2wrist[0, 3] += (self.camera_info.width / self.camera_info.width)
            # T_input2wrist[1, 3] -= (self.camera_info.height / self.camera_info.width)
            T_input2wrist[0, 3] += 0.5 * (self.camera_info.width / self.camera_info.width)
            T_input2wrist[1, 3] -= 0.5 * (self.camera_info.height / self.camera_info.width)
            
            self._send_tf(parent=input_frame, child=wrist_frame,
                        transform_matrix=T_input2wrist, stamp=stamp)

            # wrist → joints
            for (finger, jname), T in frames.T_wrist2joint.items():
                child = f"hand_{label}_{finger}_{jname}_{suffix}"
                self._send_tf(parent=wrist_frame, child=child, transform_matrix=T, stamp=stamp)

            # # (옵션) depth anchoring → norm 좌표계 wrist 픽셀 기준
            # if self.use_depth and (self.depth is not None) and (self.K is not None):
            #     u, v = float(lm_norm[0, 0]), float(lm_norm[0, 1])
            #     Z = self._depth_at(u, v, ksize=9)
            #     if np.isfinite(Z) and Z > 0:
            #         fx, fy, cx, cy = self.K
            #         p_cam = self._backproject(u, v, Z, fx, fy, cx, cy)
            #         T_cam2wrist = np.eye(4, dtype=np.float32)
            #         T_cam2wrist[:3, 3] = p_cam
            #         self._send_tf(parent=self.camera_frame,
            #                     child=wrist_frame,
            #                     transform_matrix=T_cam2wrist,
            #                     stamp=stamp)
                    
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
                _publish_hand_tfs(label, frames_norm, suffix="norm")

            # canonical 기반
            if self.tf_canon_enable:
                solver.update_landmarks(lm_canon)
                frames_canon = solver.compute(label=label)
                _publish_hand_tfs(label, frames_canon, suffix="canon")

            # canonical을 정규화
            if self.tf_canon_norm_enable:
                solver.update_landmarks(lm_canon * self.tf_canon_norm_scale)
                frames_canon = solver.compute(label=label)
                _publish_hand_tfs(label, frames_canon, suffix="canon_norm")

            # canonical_norm을 기준으로 원하는 스케일 적용
            # TBD
            # 2025.09.07
            if self.tf_user_defined_enable:
                solver.update_landmarks(lm_canon * self.tf_canon_norm_scale * self.tf_user_defined_scale)
                frames_user_defined = solver.compute(label=label)
                _publish_hand_tfs(label, frames_user_defined, suffix=self.tf_user_defined_suffix)

    def param_callback(self, params):
        for param in params:
            if param.name == 'tf.norm.enable':
                self.tf_norm_enable = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_norm_enable}")
            elif param.name == 'tf.canonical.enable':
                self.tf_canon_enable = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_canon_enable}")
            elif param.name == 'tf.canonical_norm.enable':
                self.tf_canon_norm_enable = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_canon_norm_enable}")
            elif param.name == 'tf.canonical_norm.scale':
                self.tf_canon_norm_scale = param.value
                self.get_logger().info(f"Parameter '{param.name}' updated -> {self.tf_canon_norm_scale}")

        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = HandPoseTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
