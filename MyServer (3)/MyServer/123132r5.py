#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, math, socket
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

# --- Utils ---
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def bbox_center(b):
    x1, y1, x2, y2 = b
    return (0.5 * (x1 + x2), 0.5 * (y1 + y2))

def iou_xyxy(a, b):
    a_area = (a[2] - a[0]) * (a[3] - a[1])
    b_area = (b[2] - b[0]) * (b[3] - b[1])
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    iw = max(0, ix2 - ix1); ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    return float(inter) / float(a_area + b_area - inter + 1e-6)

def order_points(pts4):
    pts = np.array(pts4, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)
    tl = pts[np.argmin(s)]; br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]; bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], dtype=np.float32)

def dist2(a, b):
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))

# --- Track Class ---
class Track:
    __slots__ = (
        "id", "bbox", "ema_bbox", "conf", "miss", "last_seen",
        "corners", "ema_corners", "theta",
        "depth_m", # Placeholder
        "size_id", "size_lock", "size_votes",
        "last_area", "clf_method",
        "is_gray", "velocity"
    )

    def __init__(self, tid, bbox, conf, tnow, vote_window=15):
        self.id = tid
        self.bbox = bbox
        self.ema_bbox = tuple(map(float, bbox))
        self.conf = float(conf)
        self.miss = 0
        self.last_seen = tnow
        self.corners = None
        self.ema_corners = None
        self.theta = 0.0
        self.depth_m = 0.0
        self.size_id = -1
        self.size_lock = -1
        self.size_votes = deque(maxlen=vote_window)
        self.last_area = 0.0
        self.clf_method = "NONE"
        self.is_gray = False
        self.velocity = (0.0, 0.0)

class REDLINE_TUNABLE(Node):
    def __init__(self):
        super().__init__("REDLINE_B0205_TUNABLE")

        # Topics
        self.model_path = "/home/dhdh/runs/detect/train3/weights/best.pt"
        self.color_topic = "/camera/camera/color/image_raw"
        
        # --- Tuning (Initial Defaults) ---
        self.roi_y_top_percent = 40
        self.roi_y_bot_percent = 100
        self.gate_x0_percent = 62
        self.gate_x1_percent = 65
        
        self.imgsz = 640
        self.conf = 0.25
        self.MIN_AREA = 1000

        # White/Gray gate
        self.white_ratio_min = 0.10
        self.roi_pad = 0.15
        self.min_contour_area = 250

        self.WHITE_S_MAX = 100
        self.WHITE_V_MIN = 160

        self.GRAY_S_MAX = 80
        self.GRAY_V_MIN = 30
        self.gray_ratio_min = 0.02

        # Classification
        self.size_tol_cm = 2.5
        self.lock_vote_need = 6
        self.vote_window = 15
        self.area_th_px = 20000 
        
        # Tracking matching
        self.match_iou_min = 0.05
        self.match_dist_px = 400.0

        # TCP & ROS
        self.win_ip, self.win_port = "192.168.0.22", 6000
        self.tcp_sock = None
        self.tcp_connected = False

        self.bridge = CvBridge()
        self.tracks = []
        self.next_id = 1
        self.gate_state = {}
        self.seq = 0
        self.queue = deque(maxlen=100)

        self._last_yolo_t = 0.0
        self._last_yolo_dets = []

        # Gate absolute positions (calculated later)
        self.gate_x0 = 0
        self.gate_x1 = 0

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.sub_color = self.create_subscription(Image, self.color_topic, self.cb_color, qos)
        
        self.pub = self.create_publisher(Float32MultiArray, "/box/tracks_size", 10)

        self.model = YOLO(self.model_path)
        self.create_timer(1.0, self._tcp_try_connect)

        self.WIN_VIEW = "REDLINE_B0205_TUNABLE"
        cv2.namedWindow(self.WIN_VIEW, cv2.WINDOW_NORMAL)
        
        # ✅ UI Sliders (Modified for Top/Bot ROI Only)
        cv2.createTrackbar("ROI_Top%", self.WIN_VIEW, self.roi_y_top_percent, 100, self._nothing)
        cv2.createTrackbar("ROI_Bot%", self.WIN_VIEW, self.roi_y_bot_percent, 100, self._nothing)
        # Gate Sliders Removed (Fixed to a0205 defaults)
        cv2.createTrackbar("AreaTh_x100", self.WIN_VIEW, int(self.area_th_px/100), 500, self._nothing)
        
        self.get_logger().info("🚀 REDLINE B0205 TUNABLE (Lite + UI Sliders) Started")

    def _nothing(self, x):
        pass

    def _update_params_from_ui(self, W):
        # 1) ROI Top/Bot
        top = cv2.getTrackbarPos("ROI_Top%", self.WIN_VIEW)
        bot = cv2.getTrackbarPos("ROI_Bot%", self.WIN_VIEW)
        
        if top >= bot:
            bot = top + 1
            cv2.setTrackbarPos("ROI_Bot%", self.WIN_VIEW, bot)
            
        self.roi_y_top_percent = top
        self.roi_y_bot_percent = bot
        
        self.roi_y_top_percent = top
        self.roi_y_bot_percent = bot
        
        # 2) Gate X0, X1 (Fixed defaults from a0205.py)
        # We rely on _ensure_gate or just set here if we want dynamic resize support
        self.gate_x0_percent = 62
        self.gate_x1_percent = 65
        
        # Update Absolute
        self.gate_x0 = int(W * 0.62)
        self.gate_x1 = int(W * 0.65)

        # 3) Area Threshold
        t = cv2.getTrackbarPos("AreaTh_x100", self.WIN_VIEW)
        if t > 0: self.area_th_px = t * 100

    # -------------------------
    # Classification
    # -------------------------
    def _classify_logic(self, tr):
        # Gray Priority
        if tr.is_gray:
            tr.size_id = 0
            tr.size_lock = -1 # Prevent lock
            tr.clf_method = "GRAY"
            return

        # Locked
        if tr.size_lock in [0, 1]:
            tr.size_id = tr.size_lock
            return

        sid = 1 if tr.last_area >= self.area_th_px else 0
        method = "AREA"

        tr.size_id = sid
        tr.clf_method = method

    # -------------------------
    # ROS callbacks
    # -------------------------
    def cb_color(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        H, W = frame.shape[:2]
        
        # ✅ Update Params First
        self._update_params_from_ui(W)
        
        t_start = time.time()
        dets = self._run_yolo(frame)
        self._update_tracks(dets, t_start)
        self._refine_tracks(frame)

        for tr in self.tracks:
            tr.depth_m = 0.0
            self._classify_logic(tr)
            self._gate_process(tr, t_start)

        self._publish()
        vis = self._draw(frame, (time.time() - t_start) * 1000)
        cv2.imshow(self.WIN_VIEW, vis)
        cv2.waitKey(1)

    # -------------------------
    # YOLO + gate
    # -------------------------
    def _run_yolo(self, frame):
        now = time.time()
        if (now - self._last_yolo_t) < 0.1:
            return self._last_yolo_dets

        H, W = frame.shape[:2]
        # ✅ Dynamic ROI (Top & Bottom)
        y_top = int(H * self.roi_y_top_percent / 100)
        y_bot = int(H * self.roi_y_bot_percent / 100)
        
        y_top = clamp(y_top, 0, H-1)
        y_bot = clamp(y_bot, y_top+1, H)
        
        roi = frame[y_top:y_bot, :]

        results = self.model.predict(roi, conf=self.conf, verbose=False, imgsz=self.imgsz)[0]

        res = []
        for b in results.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            y1 += y_top
            y2 += y_top

            area = (x2 - x1) * (y2 - y1)
            if area < self.MIN_AREA:
                continue

            r_roi = frame[int(y1):int(y2), int(x1):int(x2)]
            if r_roi.size > 0:
                hsv = cv2.cvtColor(r_roi, cv2.COLOR_BGR2HSV)

                mask_white = cv2.inRange(
                    hsv,
                    (0, 0, self.WHITE_V_MIN),
                    (180, self.WHITE_S_MAX, 255)
                )
                white_ratio = np.count_nonzero(mask_white) / (mask_white.size + 1e-6)

                mask_gray = cv2.inRange(
                    hsv,
                    (0, 0, self.GRAY_V_MIN),
                    (180, self.GRAY_S_MAX, 255)
                )
                gray_ratio = np.count_nonzero(mask_gray) / (mask_gray.size + 1e-6)

                is_gray = False
                if (white_ratio < self.white_ratio_min):
                    if (gray_ratio >= self.gray_ratio_min):
                        is_gray = True
                    else:
                        continue 
                
                if not is_gray and (white_ratio >= self.white_ratio_min):
                    is_gray = False

            res.append((float(b.conf[0]), (x1, y1, x2, y2), is_gray))

        self._last_yolo_t = now
        self._last_yolo_dets = res
        return res

    # -------------------------
    # Tracking
    # -------------------------
    def _update_tracks(self, dets, now):
        used = [False] * len(dets)
        for tr in self.tracks:
            tr.miss += 1

            best_idx = -1
            best_score = -1e9

            # Predict next position based on velocity
            tcx, tcy = bbox_center(tr.ema_bbox)
            vx, vy = tr.velocity
            px, py = tcx + vx, tcy + vy 
            
            p_bbox = (
                tr.ema_bbox[0] + vx, tr.ema_bbox[1] + vy,
                tr.ema_bbox[2] + vx, tr.ema_bbox[3] + vy
            )

            for i, (cf, bb, is_g) in enumerate(dets):
                if used[i]:
                    continue

                # Use predicted bbox for IOU
                iou = iou_xyxy(p_bbox, bb)
                dcx, dcy = bbox_center(bb)
                dist = math.hypot(dcx - px, dcy - py) 

                if iou < self.match_iou_min and dist > self.match_dist_px:
                    continue

                score = (2.0 * iou) - (dist / max(self.match_dist_px, 1e-6))
                if score > best_score:
                    best_score = score
                    best_idx = i

            if best_idx >= 0:
                used[best_idx] = True
                prev_cx, prev_cy = bbox_center(tr.ema_bbox)

                tr.bbox = dets[best_idx][1]
                tr.conf = dets[best_idx][0]
                tr.is_gray = dets[best_idx][2]
                tr.miss = 0
                tr.last_seen = now

                # Update Velocity
                curr_cx, curr_cy = bbox_center(tr.bbox)
                dx, dy = curr_cx - prev_cx, curr_cy - prev_cy
                alpha_v = 0.3
                vx = (1 - alpha_v) * tr.velocity[0] + alpha_v * dx
                vy = (1 - alpha_v) * tr.velocity[1] + alpha_v * dy
                tr.velocity = (vx, vy)

                # Update EMA Box
                a = 0.7
                tr.ema_bbox = tuple((1 - a) * e + a * v for e, v in zip(tr.ema_bbox, tr.bbox))
                
                w = tr.ema_bbox[2] - tr.ema_bbox[0]
                h = tr.ema_bbox[3] - tr.ema_bbox[1]
                tr.last_area = w * h

        # 새 트랙 생성
        for i, u in enumerate(used):
            if not u:
                new_tr = Track(self.next_id, dets[i][1], dets[i][0], now, vote_window=self.vote_window)
                new_tr.is_gray = dets[i][2]
                self.tracks.append(new_tr)
                self.next_id += 1

        self.tracks = [t for t in self.tracks if t.miss < 30]

    # -------------------------
    # Corner refine
    # -------------------------
    def _refine_tracks(self, frame):
        H, W = frame.shape[:2]
        for tr in self.tracks:
            x1, y1, x2, y2 = map(int, tr.ema_bbox)
            bw, bh = x2 - x1, y2 - y1
            px, py = int(bw * self.roi_pad), int(bh * self.roi_pad)
            X1, Y1 = max(0, x1 - px), max(0, y1 - py)
            X2, Y2 = min(W, x2 + px), min(H, y2 + py)
            roi = frame[Y1:Y2, X1:X2]
            if roi.size == 0:
                continue

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # --- Masking ---
            if tr.is_gray:
                mask = cv2.inRange(hsv, (0, 0, self.GRAY_V_MIN), (180, self.GRAY_S_MAX, 255))
            else:
                mask = cv2.inRange(hsv, (0, 0, self.WHITE_V_MIN), (180, self.WHITE_S_MAX, 255))

            # --- Denoise ---
            mask = cv2.GaussianBlur(mask, (5, 5), 0)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) 

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                if cv2.contourArea(c) > self.min_contour_area:
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = order_points(box)
                    box[:, 0] += X1
                    box[:, 1] += Y1
                    # Smooth adjustment
                    tr.ema_corners = box if tr.ema_corners is None else (0.4 * tr.ema_corners + 0.6 * box)
                    tr.theta = rect[2]

    # -------------------------
    # Gate
    # -------------------------
    def _gate_process(self, tr, now):
        cx = (tr.ema_bbox[0] + tr.ema_bbox[2]) * 0.5
        st = self.gate_state.setdefault(tr.id, {"in": False, "t": 0.0})
        
        # Check using current (dynamic) gate positions
        self._tcp_send(tr)
        if self.gate_x0 <= cx <= self.gate_x1:
            st["in"] = True
        elif st["in"] and cx > self.gate_x1:
            if (now - st["t"]) > 0.6 and tr.size_id >= 0:
                self.seq += 1
                st["t"] = now
                st["in"] = False
                self._tcp_send(tr)
                self.get_logger().info(
                    f"📦 PASS [{self.seq}] ID:{tr.id} Size:{'L' if tr.size_id==1 else 'S'} "
                    f"Method:{tr.clf_method} "
                    f"BBOX_A={int(tr.last_area)}"
                )

    # -------------------------
    # TCP
    # -------------------------
    def _tcp_try_connect(self):
        if self.tcp_connected:
            print("test1")
            return
        try:
            self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_sock.settimeout(0.2)
            self.tcp_sock.connect((self.win_ip, self.win_port))
            self.tcp_connected = True
        except Exception:
            self.tcp_connected = False

    def _tcp_send(self, tr):
        if not self.tcp_connected or self.tcp_sock is None:
            print("test2")
            return
        msg = f"PASS seq={self.seq} size={'LARGE' if tr.size_id==1 else 'SMALL'} z=0.000\n"
        print(msg)
        try:
            self.tcp_sock.sendall(msg.encode("utf-8"))
        except Exception:
            self.tcp_connected = False

    # -------------------------
    # Publish
    # -------------------------
    def _publish(self):
        msg = Float32MultiArray()
        data = [float(len(self.tracks))]
        for t in self.tracks:
            data.extend([
                float(t.id),
                float(t.ema_bbox[0]), float(t.ema_bbox[1]),
                float(t.ema_bbox[2]), float(t.ema_bbox[3]),
                float(t.size_id)
            ])
        msg.data = data
        self.pub.publish(msg)

    # -------------------------
    # Draw
    # -------------------------
    def _draw(self, frame, dt_ms):
        vis = frame.copy()
        H, W = vis.shape[:2]

        # Info Box
        cv2.rectangle(vis, (5, 5), (550, 160), (0, 0, 0), -1)
        cv2.putText(vis, f"TCP:{'ON' if self.tcp_connected else 'OFF'}  Depth:OFF",
                    (15, 30), 1, 1.1, (0, 255, 0), 2)
        cv2.putText(vis, f"AreaTh:{self.area_th_px}",
                    (15, 60), 1, 1.1, (0, 255, 255), 2)
        cv2.putText(vis, f"ROI Y:{self.roi_y_top_percent}%~{self.roi_y_bot_percent}%",
                    (15, 90), 1, 1.1, (0, 255, 255), 2)
        cv2.putText(vis, f"Match: iou>={self.match_iou_min}",
                    (15, 120), 1, 1.1, (255, 255, 255), 2)
        cv2.putText(vis, f"Gate: V>={self.WHITE_V_MIN} / {self.GRAY_V_MIN}",
                    (15, 150), 1, 1.0, (255, 200, 200), 2)

        # Draw ROI Lines
        y_top = int(H * self.roi_y_top_percent / 100)
        y_bot = int(H * self.roi_y_bot_percent / 100)
        
        cv2.line(vis, (0, y_top), (W, y_top), (255, 0, 255), 2)
        cv2.putText(vis, "ROI TOP", (10, y_top - 10), 1, 1, (255, 0, 255), 2)
        
        if y_bot < H:
            cv2.line(vis, (0, y_bot), (W, y_bot), (255, 0, 255), 2)
            cv2.putText(vis, "ROI BOT", (10, y_bot - 10), 1, 1, (255, 0, 255), 2)

        # Draw Gate Lines
        cv2.line(vis, (self.gate_x0, 0), (self.gate_x0, H), (0, 0, 255), 2)
        cv2.line(vis, (self.gate_x1, 0), (self.gate_x1, H), (0, 0, 255), 2)
        
        # Gate Zone Visualization
        if self.gate_x1 > self.gate_x0:
            overlay = vis.copy()
            cv2.rectangle(overlay, (self.gate_x0, 0), (self.gate_x1, H), (0, 0, 255), -1)
            cv2.addWeighted(overlay, 0.2, vis, 0.8, 0, vis)

        for tr in self.tracks:
            x1, y1, x2, y2 = map(int, tr.ema_bbox)
            color = (0, 255, 0) if tr.size_id == 1 else (255, 255, 0) if tr.size_id == 0 else (160, 160, 160)
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

            tag = "L" if tr.size_id == 1 else "S" if tr.size_id == 0 else "?"
            lock = "*" if tr.size_lock >= 0 else ""
            cv2.putText(vis, f"{tr.id}{tag}{lock}: {tr.clf_method}", (x1, y1 - 10), 1, 1.1, color, 2)

            dim_txt = f"Area:{int(tr.last_area)} px"
            cv2.putText(vis, dim_txt, (x1, y2 + 25), 1, 1.1, (255, 255, 255), 2)
            
            if tr.ema_corners is not None:
                cv2.polylines(vis, [tr.ema_corners.astype(np.int32)], True, (0, 0, 255), 1)

        return vis

def main():
    rclpy.init()
    node = REDLINE_TUNABLE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()