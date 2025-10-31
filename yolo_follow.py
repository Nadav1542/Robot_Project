import signal
import time
import math
import threading
import cv2
import numpy as np
from ultralytics import YOLO

# Unitree SDK imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient

# Project helpers
from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor

# New: Follow controller module
from follow_controller import FollowController, FollowConfig

from camera import Camera

from target_lock import TargetLock, TargetLockConfig



# -------------------- Config --------------------
WIN_NAME          = "GO2 Camera + YOLO (Follow + Chair)"
CAM_TIMEOUT_SEC   = 2.0

# YOLO/ROI config
MIN_CONF          = 0.35
MIN_BOX_FRAC      = 0.05
ROI_NORM = (0.33, 0.02, 0.67, 0.98)
SIZE_TOL          = 0.08
CENTER_TOL        = 0.10

# Motion control (these will also be passed to FollowConfig)
MAX_VX            = 0.40     # m/s
MAX_WZ            = 0.96     # rad/s
K_WZ              = 1.2
K_VX_FWD          = 0.8
K_VX_BACK         = 0.4
SMOOTH_ALPHA      = 0.2
FOLLOW_DT         = 0.04

# Follow controller (UWB)
DEAD_BAND_D       = 1.2
DIST_SLOWDOWN     = 1.0
DEAD_BAND_O       = 0.20
SLOWDOWN_ANGLE    = math.radians(60)
MAX_VX_FOLLOW     = 0.9
MAX_WZ_FOLLOW     = 0.96

# Target lock
LOCK_IOU_MIN      = 0.25
LOCK_MAX_MISS_FR  = 10
PREFER_ROI        = True

# Behavior timing
HOLD_SECONDS      = 3.0
COOLDOWN_SECONDS  = 8.0

# -------------------- Globals --------------------
stop_event = threading.Event()

# Shared behavior state for the motion thread
behavior = {
    "mode": "FOLLOW",       # FOLLOW, APPROACH, HOLD
    "vx": 0.0,
    "wz": 0.0,
    "until": 0.0,
    "cooldown_until": 0.0,
    "target_box": None,
    "roi_px": None,
}

# -------------------- Utilities --------------------
def handle_sigint(signum, frame):
    print("\n[SYS] Ctrl+C detected — stopping...")
    stop_event.set()




# -------------------- Main (YOLO + State machine) --------------------
def main():
    signal.signal(signal.SIGINT, handle_sigint)

    # Init Unitree comms
    ChannelFactoryInitialize(0)
    state_manager = UwbStateManager()
    button_monitor = UwbButtonMonitor(state_manager, lambda: print("[UWB] Shutting down..."))
    uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
    uwb_sub.Init(button_monitor.get_callback(), 10)

    # Movement clients
    sport = SportClient()
    avoid = ObstaclesAvoidClient()
    avoid.Init()
    sport.Init()
    avoid.UseRemoteCommandFromApi(True)
    avoid.SwitchSet(True)
    # Start background follow controller
    follow_cfg = FollowConfig(
        SMOOTH_ALPHA=SMOOTH_ALPHA,
        MAX_VX=MAX_VX,
        MAX_WZ=MAX_WZ,
        FOLLOW_DT=FOLLOW_DT,
        DEAD_BAND_D=DEAD_BAND_D,
        DIST_SLOWDOWN=DIST_SLOWDOWN,
        DEAD_BAND_O=DEAD_BAND_O,
        SLOWDOWN_ANGLE=SLOWDOWN_ANGLE,
        MAX_VX_FOLLOW=MAX_VX_FOLLOW,
        MAX_WZ_FOLLOW=MAX_WZ_FOLLOW,
    )
    follower = FollowController(state_manager, avoid, behavior, follow_cfg)
    follower.start(stop_event, daemon=True)
    print("[FOLLOW] Started background follow loop (default).")

    # Camera + YOLO
    cam = Camera(timeout_sec=CAM_TIMEOUT_SEC)
    model = YOLO("yolov8n.pt")
    names = model.model.names

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 960, 540)

    lock_cfg = TargetLockConfig(
    lock_iou_min=LOCK_IOU_MIN,
    lock_max_miss_fr=LOCK_MAX_MISS_FR,
    prefer_roi=PREFER_ROI,
)
    lock = TargetLock(lock_cfg)

    fps_t0, frames = time.time(), 0
    hold_until = 0.0
    last_announce = 0.0

    try:
        while not stop_event.is_set():
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.01)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            h, w = frame.shape[:2]

            # ROI in px
            rx1 = int(ROI_NORM[0] * w)
            ry1 = int(ROI_NORM[1] * h)
            rx2 = int(ROI_NORM[2] * w)
            ry2 = int(ROI_NORM[3] * h)
            roi_w = float(rx2 - rx1)
            roi_h = float(ry2 - ry1)
            roi_cx = 0.5 * (rx1 + rx2)

            # draw ROI (green if locked/approaching/holding, else white)
            roi_col = (0, 255, 0) if behavior["mode"] in ("APPROACH", "HOLD") else (255, 255, 255)
            cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), roi_col, 2)

            # YOLO inference
            res = model.predict(frame, imgsz=640, conf=MIN_CONF, verbose=False)[0]

            # Collect chair candidates
            candidates = []
            if hasattr(res, 'boxes') and res.boxes is not None:
                boxes = res.boxes.xyxy.cpu().numpy()
                clss  = res.boxes.cls.cpu().numpy().astype(int)
                confs = res.boxes.conf.cpu().numpy()
                for (x1, y1, x2, y2), cid, p in zip(boxes, clss, confs):
                    if names.get(int(cid), "") != "potted plant" or p < MIN_CONF:
                        continue
                    if (y2 - y1) < (MIN_BOX_FRAC * h):
                        continue
                    candidates.append((float(p), (float(x1), float(y1), float(x2), float(y2))))

            now = time.time()

            # --- State machine ---
            mode = behavior["mode"]

            # Update ROI in shared behavior (used by motion thread for awareness)
            behavior["roi_px"] = (rx1, ry1, rx2, ry2)

            if mode == "FOLLOW":
                # Ready to acquire only if cooldown passed
                if now >= behavior["cooldown_until"]:
                    # Try to acquire a target if we have candidates
                    if not lock.active and candidates:
                        got = lock.acquire(candidates, roi_rect=(rx1, ry1, rx2, ry2))
                        if got:
                            behavior["mode"] = "APPROACH"
                            behavior["target_box"] = lock.box
                            behavior["vx"] = 0.0
                            behavior["wz"] = 0.0

            elif mode == "APPROACH":
                # Maintain/update lock, if lost -> back to FOLLOW
                if lock.active:
                    lock.update(candidates)
                if not lock.active or lock.box is None:
                    behavior["mode"] = "FOLLOW"
                    behavior["target_box"] = None
                else:
                    # Compute control towards the locked chair
                    x1, y1, x2, y2 = lock.box
                    cx = 0.5 * (x1 + x2)
                    bh = float(y2 - y1)
                    ex = (cx - roi_cx) / max(roi_w, 1.0)   # left<0, right>0
                    size_ratio = bh / max(roi_h, 1.0)
                    ey = 1.0 - size_ratio                  # >0: need to get closer

                    # yaw
                    if abs(ex) < CENTER_TOL:
                        wz_t = 0.0
                    else:
                        wz_t = -ex

                    # forward/back
                    if abs(ey) < SIZE_TOL:
                        behavior["vx"] = 0.0
                        behavior["wz"] = 0.0
                        hold_until = now + HOLD_SECONDS
                        behavior["mode"] = "HOLD"
                        print("[APPROACH] Target reached → HOLD")
                    elif ey > 0.0:
                        behavior["vx"] = K_VX_FWD * min(ey, 1.0)
                        behavior["wz"] = max(-MAX_WZ, min(MAX_WZ, wz_t))
                    else:
                        behavior["vx"] = -K_VX_BACK * min(-ey, 1.0)
                        behavior["wz"] = max(-MAX_WZ, min(MAX_WZ, wz_t))

                    behavior["target_box"] = lock.box
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 3)

            elif mode == "HOLD":
                behavior["vx"] = 0.0
                behavior["wz"] = 0.0
                if now - last_announce >= 1.0:
                    print("Found chair — holding position…")
                    sport.Hello() ## trying maybe sport.WiggleHips()
                    last_announce = now
                if now >= hold_until:
                    print("Found chair — returning to follow.")
                    behavior["mode"] = "FOLLOW"
                    behavior["cooldown_until"] = now + COOLDOWN_SECONDS
                    lock.reset()
                    behavior["target_box"] = None

            # Draw all detections + FPS
            if res is not None and res.boxes is not None:
                for b in res.boxes.xyxy.cpu().numpy():
                    x1, y1, x2, y2 = map(int, b)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

            frames += 1
            if now - fps_t0 >= 1.0:
                fps = frames / (now - fps_t0)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2, cv2.LINE_AA)
                fps_t0 = now
                frames = 0

            cv2.imshow(WIN_NAME, frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_event.set()
        
        avoid.Move(0.0, 0.0, 0.0)
        
        avoid.UseRemoteCommandFromApi(False)
        
        cam.close()
       
        cv2.destroyAllWindows()
       
        # Optionally join the controller thread before exit
        
        follower.join(timeout=1.0)
        
        print("[SYS] Shutdown complete.")


if __name__ == "__main__":
    main()



	
