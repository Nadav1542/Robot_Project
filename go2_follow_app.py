# Comments in English only
import signal
import time
import threading
import cv2
import math
import numpy as np

# Unitree SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

# Project modules
from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor
from follow_controller import FollowController, FollowConfig
from camera import Camera
from target_lock import TargetLock, TargetLockConfig
from detector import YoloDetector, DetectorConfig
from behavior_controller import BehaviorController, BehaviorConfig
from roi_utils import compute_roi, draw_roi


class Go2FollowApp:
    def __init__(self):
        self.stop_event = threading.Event()

        # Shared behavior state
        self.behavior = {
            "mode": "FOLLOW",
            "vx": 0.0,
            "wz": 0.0,
            "until": 0.0,
            "cooldown_until": 0.0,
            "target_box": None,
            "roi_px": None,
        }

        # --- Config constants ---
        self.WIN_NAME = "GO2 Camera + YOLO (Follow + Chair)"
        self.ROI_NORM = (0.33, 0.12, 0.67, 0.88)
        self.MIN_CONF = 0.35
        self.MIN_BOX_FRAC = 0.05

        # motion / control
        self.MAX_VX, self.MAX_WZ = 0.40, 0.96
        self.K_WZ, self.K_VX_FWD, self.K_VX_BACK = 1.2, 0.8, 0.4
        self.CENTER_TOL, self.SIZE_TOL = 0.10, 0.08
        self.SMOOTH_ALPHA, self.FOLLOW_DT = 0.2, 0.04

        # Follow (UWB)
        self.DEAD_BAND_D, self.DIST_SLOWDOWN = 1.2, 1.0
        self.DEAD_BAND_O, self.SLOWDOWN_ANGLE = 0.20, math.radians(60)
        self.MAX_VX_FOLLOW, self.MAX_WZ_FOLLOW = 0.9, 0.96

        # Target lock
        self.LOCK_IOU_MIN, self.LOCK_MAX_MISS_FR, self.PREFER_ROI = 0.25, 10, True

        # Behavior timing
        self.HOLD_SECONDS, self.COOLDOWN_SECONDS = 3.0, 8.0

    # -------------------- Setup --------------------
    def setup_robot(self):
        # Initialize communication
        ChannelFactoryInitialize(0)
        self.state_manager = UwbStateManager()
        self.button_monitor = UwbButtonMonitor(
            self.state_manager, lambda: print("[UWB] Shutting down...")
        )
        self.uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
        self.uwb_sub.Init(self.button_monitor.get_callback(), 10)

        # Movement clients
        self.sport = SportClient()
        self.avoid = ObstaclesAvoidClient()
        self.avoid.Init()
        self.sport.Init()
        self.avoid.UseRemoteCommandFromApi(True)
        self.avoid.SwitchSet(True)
        print("[SETUP] Robot communication initialized.")

    def setup_modules(self):
        # Follow controller
        follow_cfg = FollowConfig(
            SMOOTH_ALPHA=self.SMOOTH_ALPHA,
            MAX_VX=self.MAX_VX,
            MAX_WZ=self.MAX_WZ,
            FOLLOW_DT=self.FOLLOW_DT,
            DEAD_BAND_D=self.DEAD_BAND_D,
            DIST_SLOWDOWN=self.DIST_SLOWDOWN,
            DEAD_BAND_O=self.DEAD_BAND_O,
            SLOWDOWN_ANGLE=self.SLOWDOWN_ANGLE,
            MAX_VX_FOLLOW=self.MAX_VX_FOLLOW,
            MAX_WZ_FOLLOW=self.MAX_WZ_FOLLOW,
        )
        self.follower = FollowController(
            self.state_manager, self.avoid, self.behavior, follow_cfg
        )
        self.follower.start(self.stop_event, daemon=True)

        # Vision modules
        self.cam = Camera(timeout_sec=2.0)
        self.detector = YoloDetector(
            weights="yolov8n.pt",
            cfg=DetectorConfig(
                min_conf=self.MIN_CONF,
                imgsz=640,
                label="chair",
                min_box_frac=self.MIN_BOX_FRAC,
            ),
        )

        # Target lock + behavior
        self.lock = TargetLock(
            TargetLockConfig(
                self.LOCK_IOU_MIN, self.LOCK_MAX_MISS_FR, self.PREFER_ROI
            )
        )
        self.beh = BehaviorController(
            self.behavior,
            self.lock,
            BehaviorConfig(
                K_WZ=self.K_WZ,
                K_VX_FWD=self.K_VX_FWD,
                K_VX_BACK=self.K_VX_BACK,
                CENTER_TOL=self.CENTER_TOL,
                SIZE_TOL=self.SIZE_TOL,
                MAX_WZ=self.MAX_WZ,
                HOLD_SECONDS=self.HOLD_SECONDS,
                COOLDOWN_SECONDS=self.COOLDOWN_SECONDS,
            ),
        )
        print("[SETUP] Modules initialized.")

    # -------------------- Run --------------------
    def run(self):
        self.setup_robot()
        self.setup_modules()

        cv2.namedWindow(self.WIN_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WIN_NAME, 960, 540)

        fps_t0, frames = time.time(), 0

        try:
            while not self.stop_event.is_set():
                frame = self.cam.get_frame()
                if frame is None:
                    time.sleep(0.01)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
                    continue

                # ROI compute + draw
                roi_rect, roi_geom = compute_roi(frame.shape, self.ROI_NORM)
                draw_roi(frame, roi_rect, self.behavior["mode"])

                # Detection + behavior
                candidates, res = self.detector.detect(frame)
                now = time.time()
                draw_box = self.beh.step(
                    roi_px=roi_rect,
                    roi_geometry=roi_geom,
                    candidates=candidates,
                    now=now,
                )

                # Highlight target if locked
                if draw_box is not None:
                    x1, y1, x2, y2 = draw_box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 3)

                # Draw all detections
                YoloDetector.draw_all_boxes(frame, res)

                # FPS overlay
                frames += 1
                if now - fps_t0 >= 1.0:
                    fps = frames / (now - fps_t0)
                    cv2.putText(
                        frame,
                        f"FPS: {fps:.1f}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    fps_t0, frames = now, 0

                cv2.imshow(self.WIN_NAME, frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        finally:
            self.shutdown()

    # -------------------- Shutdown --------------------
    def shutdown(self):
        print("[SYS] Shutting down...")
        self.stop_event.set()
        try:
            self.avoid.Move(0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            self.avoid.UseRemoteCommandFromApi(False)
            self.avoid.SwitchSet(False)
        except Exception:
            pass
        self.cam.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            self.follower.join(timeout=1.0)
        except Exception:
            pass
        print("[SYS] Shutdown complete.")
