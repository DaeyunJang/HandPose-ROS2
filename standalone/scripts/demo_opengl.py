# run_qt.py
# pip install pyqt5 pyqtgraph

import sys, os, numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph import Vector

# --- Qt 플러그인 경로 오염 방지 (opencv-headless 환경 호환) ---
os.environ.pop("QT_PLUGIN_PATH", None)
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

# --- 공용 설정 로더 ---
from hand_config_loader import get_config

_CFG = get_config()
# === 공통 상수/설정 ===
FINGERS = {k: list(v) for k, v in _CFG["fingers"].items()}
PALM_CONNECTIONS = [tuple(x) for x in _CFG["palm_connections"]]
TIP_IDS = list(_CFG["tip_ids"])
FINGER_COLORS = {k: tuple(v) for k, v in _CFG["finger_color_rgba"].items()}  # RGBA 0~1
VIEWER = _CFG["viewer"]
VW_WRIST = VIEWER["wrist"]
VW_INPUT = VIEWER["input"]
DEFAULTS = _CFG["defaults"]
FLIP_X = bool(DEFAULTS["flip_x"])
CAMERA = DEFAULTS["camera"]
WIDTH  = int(CAMERA["width"])
HEIGHT = int(CAMERA["height"])
FPS    = int(CAMERA["fps"])


# === 깊이 컬러맵 ===
CMAP = pg.colormap.get('viridis')

def apply_affine(T, pts):
    """4x4 T, (N,3) pts -> (N,3)"""
    R = T[:3,:3]; t = T[:3,3]
    return (pts @ R.T) + t

class Worker(QtCore.QThread):
    """
    updated emits: dict with
      {
        "hands": [
          {
            "label": "left"/"right",
            # WRIST frame
            "lm_w": (21,3) float32,
            "wrist_axes_w": {"o":(3,), "X":(3,), "Y":(3,), "Z":(3,)},
            "joint_o_w": (M,3) float32,
            "joint_xyz_w": (M,3,3) float32,

            # INPUT frame
            "lm_in": (21,3) float32,
            "wrist_axes_in": {"o":(3,), "X":(3,), "Y":(3,), "Z":(3,)},
            "joint_o_in": (M,3) float32,
            "joint_xyz_in": (M,3,3) float32,

            # 2D overlay
            "lm_canon": (21,3) float32
          }, ...
        ],
        "preview_rgb": HxWx3 uint8 (RGB),
        "size": (width, height)
      }
    """
    updated = QtCore.pyqtSignal(dict)

    def __init__(self, width=WIDTH, height=HEIGHT, fps=FPS, parent=None):
        super().__init__(parent)
        self.width, self.height, self.fps = width, height, fps
        self.running = True
        self.flip_view = True  # 미러링 프리뷰(필요 시 False)
        self.cam = None
        self.est = None
        self.solvers = None

    def run(self):
        # 지연 import (워커 스레드 내부)
        from realsense_camera import RealSenseCamera
        from mediapipe_hands import MediaPipeHandEstimator
        from landmark_to_handpose import LandmarkToHandPose
        import cv2

        self.cam = RealSenseCamera(width=self.width, height=self.height, fps=self.fps,
                                   enable_color=True, enable_depth=False, align_to_color=True)
        self.est = MediaPipeHandEstimator(static_image_mode=False, max_num_hands=2,
                                          min_detection_confidence=0.6, min_tracking_confidence=0.6)

        # config의 flip_x 적용
        self.solvers = {
            "left":  LandmarkToHandPose(flip_x=FLIP_X, hand_label="left"),
            "right": LandmarkToHandPose(flip_x=FLIP_X, hand_label="right"),
        }

        self.cam.start()
        try:
            while self.running:
                color_bgr, _, _ = self.cam.get_frames()
                if color_bgr is None:
                    continue
                if self.flip_view:
                    color_bgr = cv2.flip(color_bgr, 1)

                ok, handed, lm_norm_list, lm_canon_list = self.est.detect(color_bgr)

                preview_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
                H, W = preview_rgb.shape[:2]
                hands_out = []

                if ok:
                    for i in range(len(lm_canon_list)):
                        lm_canon = lm_canon_list[i].astype(np.float32)  # (21,3), input/canonical
                        label = "left"
                        if i < len(handed) and handed[i].get("label","").lower() in ("left","right"):
                            label = handed[i]["label"].lower()

                        solver = self.solvers[label]
                        solver.update_landmarks(lm_canon)
                        frames = solver.compute(label=label)

                        # WRIST frame
                        lm_w = frames.landmarks_wrist.astype(np.float32)
                        wrist_axes_w = {
                            "o": np.array([0.0,0.0,0.0], np.float32),
                            "X": np.array([1.0,0.0,0.0], np.float32),
                            "Y": np.array([0.0,1.0,0.0], np.float32),
                            "Z": np.array([0.0,0.0,1.0], np.float32),
                        }
                        keys = list(frames.T_wrist2joint.keys())
                        M = len(keys)
                        joint_o_w = np.empty((M,3), np.float32)
                        joint_xyz_w = np.empty((M,3,3), np.float32)
                        for idx, T in enumerate(frames.T_wrist2joint.values()):
                            joint_o_w[idx]     = T[:3,3]
                            joint_xyz_w[idx,0] = T[:3,0]
                            joint_xyz_w[idx,1] = T[:3,1]
                            joint_xyz_w[idx,2] = T[:3,2]

                        # INPUT frame: T_input2wrist
                        T_i2w = getattr(frames, "T_input2wrist", None)
                        if T_i2w is None:
                            T_i2w = getattr(frames, "T_world2wrist", None)
                        if T_i2w is None:
                            T_i2w = np.eye(4, dtype=np.float32)
                        wrist_axes_in = {
                            "o": T_i2w[:3,3].astype(np.float32),
                            "X": T_i2w[:3,0].astype(np.float32),
                            "Y": T_i2w[:3,1].astype(np.float32),
                            "Z": T_i2w[:3,2].astype(np.float32),
                        }
                        lm_in = apply_affine(T_i2w, lm_w).astype(np.float32)
                        R_i2w = T_i2w[:3,:3].astype(np.float32); t_i2w = T_i2w[:3,3].astype(np.float32)
                        joint_o_in = (joint_o_w @ R_i2w.T) + t_i2w
                        joint_xyz_in = np.einsum('ij,mkj->mki', R_i2w, joint_xyz_w)

                        hands_out.append({
                            "label": label,
                            "lm_w": lm_w,
                            "wrist_axes_w": wrist_axes_w,
                            "joint_o_w": joint_o_w,
                            "joint_xyz_w": joint_xyz_w,
                            "lm_in": lm_in,
                            "wrist_axes_in": wrist_axes_in,
                            "joint_o_in": joint_o_in,
                            "joint_xyz_in": joint_xyz_in,
                            "lm_canon": lm_canon,
                        })

                self.updated.emit({"hands": hands_out, "preview_rgb": preview_rgb, "size": (W, H)})
        finally:
            self.cam.stop()

    def stop(self):
        self.running = False
        self.wait(500)


class HandPanel(gl.GLViewWidget):
    """
    최적화된 3D 패널: 점/팁 + 모든 선(손가락+손바닥) 1개 라인 + 손목축 3개 + 관절축(X/Y/Z) 각 1개
    - distance/grid/axis_len/joint_axis_len은 config.viewer에서 주입
    - center_pos: 카메라가 바라보는 중심점 (WRIST: (0,0,0), INPUT: (W/2,H/2,0) 권장)
    """
    def __init__(self, title="Hand",
                 distance=0.6, grid_scale=0.05, axis_len=0.05, joint_axis_len=0.025,
                 center_pos=(0,0,0), parent=None):
        super().__init__(parent)
        self.opts['distance'] = float(distance)
        self.axis_len = float(axis_len)
        self.joint_axis_len = float(joint_axis_len)

        self.setCameraPosition(elevation=20, azimuth=45)
        if center_pos is not None:
            self.setCameraPosition(pos=Vector(*center_pos))
        else:
            self.setCameraPosition(pos=Vector(0,0,0))
        self.setWindowTitle(title)

        # Grid
        g = gl.GLGridItem(); g.scale(grid_scale, grid_scale, grid_scale)
        if center_pos is not None:
            g.translate(*center_pos)
        self.addItem(g)

        # Points
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((0,3)), size=4, pxMode=True, color=(0,0,0,1))
        self.addItem(self.scatter)
        self.tips = gl.GLScatterPlotItem(pos=np.zeros((0,3)), size=10, pxMode=True, color=(1,1,1,1))
        self.addItem(self.tips)

        # All lines (fingers + palm)
        self.lines_all = gl.GLLinePlotItem(mode='lines')
        self.addItem(self.lines_all)

        # Wrist axes (3개)
        self.axis_x = gl.GLLinePlotItem(mode='lines'); self.addItem(self.axis_x)
        self.axis_y = gl.GLLinePlotItem(mode='lines'); self.addItem(self.axis_y)
        self.axis_z = gl.GLLinePlotItem(mode='lines'); self.addItem(self.axis_z)

        # Joint mini-axes (X/Y/Z 각각 1개)
        self.joint_x = gl.GLLinePlotItem(mode='lines'); self.addItem(self.joint_x)
        self.joint_y = gl.GLLinePlotItem(mode='lines'); self.addItem(self.joint_y)
        self.joint_z = gl.GLLinePlotItem(mode='lines'); self.addItem(self.joint_z)

    def _depth_colors(self, z, zmin=None, zmax=None):
        z = np.asarray(z, dtype=float)
        if z.size == 0: return np.zeros((0,4), float)
        if zmin is None: zmin = float(np.min(z))
        if zmax is None: zmax = float(np.max(z))
        if abs(zmax - zmin) < 1e-6: z_norm = np.zeros_like(z, dtype=float)
        else: z_norm = (z - zmin) / (zmax - zmin)
        return CMAP.map(z_norm, mode='float')

    def clear_draw(self):
        zero = np.zeros((0,3), np.float32)
        self.scatter.setData(pos=zero)
        self.tips.setData(pos=zero)
        self.lines_all.setData(pos=zero)
        self.axis_x.setData(pos=zero); self.axis_y.setData(pos=zero); self.axis_z.setData(pos=zero)
        self.joint_x.setData(pos=zero); self.joint_y.setData(pos=zero); self.joint_z.setData(pos=zero)

    def draw_hand(self, lm, wrist_axes, joint_o, joint_xyz):
        # Points (깊이 컬러맵)
        z = lm[:,2]; cols = self._depth_colors(z)
        self.scatter.setData(pos=lm, color=cols, size=4, pxMode=True)
        self.tips.setData(pos=lm[TIP_IDS], color=(1,1,1,1), size=10, pxMode=True)

        # All lines (fingers + palm) → (2K,3) + per-vertex color
        segs = []
        for fname, idxs in FINGERS.items():
            for a, b in zip(idxs[:-1], idxs[1:]):
                segs.append(lm[a]); segs.append(lm[b])
        for (s,e) in PALM_CONNECTIONS:
            segs.append(lm[s]); segs.append(lm[e])

        if segs:
            segs = np.asarray(segs, dtype=np.float32).reshape(-1,3)
            c2 = self._depth_colors(segs[:,2])
            self.lines_all.setData(pos=segs, color=c2, width=1.8)
        else:
            self.lines_all.setData(pos=np.zeros((0,3), np.float32))

        # Wrist axes
        o, X, Y, Z = wrist_axes["o"], wrist_axes["X"], wrist_axes["Y"], wrist_axes["Z"]
        L = self.axis_len
        self.axis_x.setData(pos=np.vstack([o, o+X*L]), color=(1,0,0,1), width=2.0)
        self.axis_y.setData(pos=np.vstack([o, o+Y*L]), color=(0,1,0,1), width=2.0)
        self.axis_z.setData(pos=np.vstack([o, o+Z*L]), color=(0,0,1,1), width=2.0)

        # Joint mini-axes
        if joint_o.size:
            Ls = self.joint_axis_len
            ex = joint_xyz[:,0,:]; Xsegs = np.empty((joint_o.shape[0]*2, 3), np.float32)
            Xsegs[0::2] = joint_o; Xsegs[1::2] = joint_o + ex * Ls
            self.joint_x.setData(pos=Xsegs, color=(1,0,0,1), width=1.2)

            ey = joint_xyz[:,1,:]; Ysegs = np.empty((joint_o.shape[0]*2, 3), np.float32)
            Ysegs[0::2] = joint_o; Ysegs[1::2] = joint_o + ey * Ls
            self.joint_y.setData(pos=Ysegs, color=(0,1,0,1), width=1.2)

            ez = joint_xyz[:,2,:]; Zsegs = np.empty((joint_o.shape[0]*2, 3), np.float32)
            Zsegs[0::2] = joint_o; Zsegs[1::2] = joint_o + ez * Ls
            self.joint_z.setData(pos=Zsegs, color=(0,0,1,1), width=1.2)
        else:
            zero = np.zeros((0,3), np.float32)
            self.joint_x.setData(pos=zero); self.joint_y.setData(pos=zero); self.joint_z.setData(pos=zero)


class PreviewPanel(QtWidgets.QWidget):
    """가벼운 2D 프리뷰 (GraphicsLayoutWidget + ImageItem)"""
    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self); lay.setContentsMargins(0,0,0,0)
        self.glw = pg.GraphicsLayoutWidget()
        self.vb = self.glw.addViewBox(lockAspect=True); self.vb.invertY(True)
        self.img = pg.ImageItem(); self.vb.addItem(self.img)

        self.max_hands = 2
        self.scatter_items = []
        self.line_items = []
        for _ in range(self.max_hands):
            sp = pg.ScatterPlotItem(size=5, pen=None, brush=pg.mkBrush(0,255,0,180))
            self.vb.addItem(sp); self.scatter_items.append(sp)
            lines = []
            for _ in PALM_CONNECTIONS:
                ln = pg.PlotDataItem(pen=pg.mkPen(200,200,200,180, width=1)); self.vb.addItem(ln); lines.append(ln)
            for _ in FINGERS:
                ln = pg.PlotDataItem(pen=pg.mkPen(0,255,0,180, width=1)); self.vb.addItem(ln); lines.append(ln)
            self.line_items.append(lines)
        lay.addWidget(self.glw)

    def show_frame(self, rgb, hands):
        if rgb is not None:
            self.img.setImage(rgb.transpose(1,0,2), levels=(0, 255))
        for sp in self.scatter_items: sp.setData([], [])
        for lines in self.line_items:
            for ln in lines: ln.setData([], [])
        for i, h in enumerate(hands[:self.max_hands]):
            lm = h.get("lm_canon", None)
            if lm is None: continue
            x = lm[:,0]; y = lm[:,1]
            self.scatter_items[i].setData(x, y)
            li = 0
            for (s,e) in PALM_CONNECTIONS:
                xs = [lm[s,0], lm[e,0]]; ys = [lm[s,1], lm[e,1]]
                self.line_items[i][li].setData(xs, ys); li += 1
            for _, idxs in FINGERS.items():
                pts = lm[idxs, :]
                self.line_items[i][li].setData(pts[:,0], pts[:,1]); li += 1


class HandViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hands: WRIST(top) + INPUT(bottom) per hand + Preview (config-driven)")
        self.resize(1900, 1000)

        central = QtWidgets.QWidget(); root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(6,6,6,6); root.setSpacing(6)

        # INPUT 뷰 카메라 중심 초기값: (W/2, H/2, 0). 실제 해상도 수신 후 on_updated에서 재설정
        self.input_center = (WIDTH/2.0, HEIGHT/2.0, 0.0)

        # 왼손: WRIST / INPUT
        left_col = QtWidgets.QVBoxLayout(); left_col.setSpacing(6)
        self.left_wrist = HandPanel("Left (WRIST)", **VW_WRIST, center_pos=(0,0,0))
        self.left_input = HandPanel("Left (INPUT)", **VW_INPUT, center_pos=(0,0,0))
        left_w = QtWidgets.QWidget(); left_w.setLayout(left_col)
        left_col.addWidget(self.left_wrist, 1); left_col.addWidget(self.left_input, 1)

        # 오른손: WRIST / INPUT
        right_col = QtWidgets.QVBoxLayout(); right_col.setSpacing(6)
        self.right_wrist = HandPanel("Right (WRIST)", **VW_WRIST, center_pos=(0,0,0))
        # self.right_input = HandPanel("Right (INPUT)", **VW_INPUT, center_pos=self.input_center)
        self.right_input = HandPanel("Right (INPUT)", **VW_INPUT, center_pos=(0,0,0))
        right_w = QtWidgets.QWidget(); right_w.setLayout(right_col)
        right_col.addWidget(self.right_wrist, 1); right_col.addWidget(self.right_input, 1)

        # 2D 프리뷰
        self.preview = PreviewPanel()

        root.addWidget(left_w,  1)
        root.addWidget(right_w, 1)
        root.addWidget(self.preview, 1)
        self.setCentralWidget(central)

        # 워커
        self.latest = {"hands": [], "preview_rgb": None, "size": (WIDTH, HEIGHT)}
        self.worker = Worker(width=WIDTH, height=HEIGHT, fps=FPS)
        self.worker.updated.connect(self.on_updated, QtCore.Qt.QueuedConnection)
        self.worker.start()

        # UI 타이머
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.redraw)
        self.timer.start(16)

    @QtCore.pyqtSlot(dict)
    def on_updated(self, payload):
        W, H = payload.get("size", self.latest.get("size", (WIDTH, HEIGHT)))
        if (W, H) != self.latest.get("size", (WIDTH, HEIGHT)):
            center = (W/2.0, H/2.0, 0.0)
            self.left_input.setCameraPosition(pos=Vector(*center))
            self.right_input.setCameraPosition(pos=Vector(*center))
        self.latest = payload

    def redraw(self):
        hands = self.latest.get("hands", [])
        rgb   = self.latest.get("preview_rgb", None)

        for panel in (self.left_wrist, self.left_input, self.right_wrist, self.right_input):
            panel.clear_draw()

        for h in hands:
            if h["label"] == "left":
                self.left_wrist.draw_hand(h["lm_w"],   h["wrist_axes_w"],  h["joint_o_w"],  h["joint_xyz_w"])
                self.left_input.draw_hand(h["lm_in"],  h["wrist_axes_in"], h["joint_o_in"], h["joint_xyz_in"])
            elif h["label"] == "right":
                self.right_wrist.draw_hand(h["lm_w"],  h["wrist_axes_w"],  h["joint_o_w"],  h["joint_xyz_w"])
                self.right_input.draw_hand(h["lm_in"], h["wrist_axes_in"], h["joint_o_in"], h["joint_xyz_in"])

        self.preview.show_frame(rgb, hands)

    def closeEvent(self, e):
        self.worker.stop()
        return super().closeEvent(e)


if __name__ == "__main__":
    pg.setConfigOptions(antialias=False)  # 성능↑
    app = QtWidgets.QApplication(sys.argv)
    w = HandViewer(); w.show()
    sys.exit(app.exec_())
