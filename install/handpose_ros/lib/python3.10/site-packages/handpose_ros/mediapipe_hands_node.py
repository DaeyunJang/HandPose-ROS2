import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np

from handpose_interfaces.msg import Hands, HandLandmarks

# mediapipe
import mediapipe as mp
try:
    import cv2
    _HAS_CV2 = True
except Exception:
    _HAS_CV2 = False

class MediaPipeHandsNode(Node):
    def __init__(self):
        super().__init__('mediapipe_hands_node')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('max_num_hands', 2)
        self.declare_parameter('min_detection_confidence', 0.8)
        self.declare_parameter('min_tracking_confidence', 0.8)
        self.declare_parameter('draw', False)  # 디버그용
        self.declare_parameter('flip_image', True)   # flip image

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.max_hands = self.get_parameter('max_num_hands').get_parameter_value().integer_value
        self.det_conf = float(self.get_parameter('min_detection_confidence').value)
        self.trk_conf = float(self.get_parameter('min_tracking_confidence').value)
        self.draw = bool(self.get_parameter('draw').value)
        self.flip = bool(self.get_parameter('flip_image').value)  # ★ 추가

        self.pub = self.create_publisher(Hands, 'hands/detections', 1)
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, self.image_topic, self.cb_image, 1)

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
        
        self.get_logger().info(f"[mediapipe] subscribe: {self.image_topic}")
        self.get_logger().info(f"[mediapipe] MediaPipiHandsNode is created.")

    @staticmethod
    def _norm_to_canonical(lm_norm: np.ndarray, w: int, h: int) -> np.ndarray:
        lm = lm_norm.copy()
        lm[:,0] = lm[:,0] * float(w)
        lm[:,1] = lm[:,1] * float(h)
        lm[:,2] = lm[:,2] * float(w)   # z는 W 기준 스케일
        return lm

    def cb_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        if bgr is None:
            return
        
        # ★ 좌우 플립(입력 단계에서 처리)
        if self.flip:
            if _HAS_CV2:
                bgr = cv2.flip(bgr, 1)     # 1 = horizontal
            else:
                bgr = bgr[:, ::-1]         # no cv2
                
        h, w = bgr.shape[:2]

        image = bgr[..., ::-1].copy()
        annotated_image = image.copy()
        res = self.hands.process(image)

        out = Hands()
        out.header = msg.header

        if not res.multi_hand_landmarks:
            self.pub.publish(out)
            
            annotated_image = cv2.flip(annotated_image, 1)  # re flip
            overlay_bgr = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
            self.pub_mp_overlay_image.publish(self.bridge.cv2_to_imgmsg(overlay_bgr))   # convert ROS message type
            return

        # Handedness
        handed_list = []
        if res.multi_handedness:
            for hnd in res.multi_handedness:
                cat = hnd.classification[0]
                handed_list.append((cat.label.lower(), float(cat.score), int(cat.index)))
        else:
            handed_list = [("right", 1.0, -1)] * len(res.multi_hand_landmarks)

        for i, lms in enumerate(res.multi_hand_landmarks):
            # normalized
            lm_norm = np.array([[p.x, p.y, p.z] for p in lms.landmark], dtype=np.float32)
            # canonical (pixel)
            lm_canon = self._norm_to_canonical(lm_norm, w, h)

            # world (meter, origin=hand center)
            if res.multi_hand_world_landmarks and i < len(res.multi_hand_world_landmarks):
                lm_world = np.array([[p.x, p.y, p.z] for p in res.multi_hand_world_landmarks[i].landmark],
                                    dtype=np.float32)
            else:
                lm_world = np.array([], dtype=np.float32)

            label, score, idx = handed_list[i] if i < len(handed_list) else ("unknown", 0.0, -1)

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
            
            self.mp_drawing.draw_landmarks(
                image=annotated_image,
                landmark_list=lms,
                connections=self.mp_hands.HAND_CONNECTIONS,
                landmark_drawing_spec=self.mp_styles.get_default_hand_landmarks_style(),
                connection_drawing_spec=self.mp_styles.get_default_hand_connections_style()
            )

            # print(f"flip_flag: {self.flip} / {m.label}")
        self.pub.publish(out)
        
        annotated_image = cv2.flip(annotated_image, 1)  # re flip
        overlay_bgr = cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR)
        self.pub_mp_overlay_image.publish(self.bridge.cv2_to_imgmsg(overlay_bgr))   # convert ROS message type

def main():
    rclpy.init()
    node = MediaPipeHandsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
