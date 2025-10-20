# Comments in English only
import time
import signal
import math
import threading
import cv2
import numpy as np

from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor


# ---------- Config ----------
CAM_TIMEOUT_SEC = 2.0
WIN_NAME = "GO2 Camera"


# ---------- Follow loop (runs in a background thread) ----------
def follow_loop(state_manager: UwbStateManager,
                avoid_client: ObstaclesAvoidClient,
                stop_event: threading.Event):
    """
    Robot moves while trying to keep distance/orientation to the UWB remote.
    Exits cleanly when stop_event is set.
    """
    # Distance control
    MAX_VX = 0.9
    DEAD_BAND_D = 1.2
    DIST_SLOWDOWN = 1.0

    # Orientation control
    MAX_WZ = 0.96
    DEAD_BAND_O = 0.20
    SLOWDOWN_ANGLE = math.radians(60)

    DT = 0.04  # 25 Hz

    while not stop_event.is_set():
        # Read UWB estimates (may be None or stale early on; guard it)
        dis = getattr(state_manager.remote_state, "distance_est", None)
        ori = getattr(state_manager.remote_state, "orientation_est", None)

        if dis is None or ori is None:
            time.sleep(DT)
            continue

        # Distance error
        err_d = dis
        if abs(err_d) <= DEAD_BAND_D:
            vx = 0.0
        else:
            scale = min(abs(err_d) / DIST_SLOWDOWN, 1.0)
            vx = math.copysign(MAX_VX * scale, err_d)
            vx = max(-MAX_VX, min(MAX_VX, vx))

        # Orientation error
        err_o = ori
        if abs(err_o) <= DEAD_BAND_O:
            wz = 0.0
        else:
            scale = min(abs(err_o) / SLOWDOWN_ANGLE, 1.0)
            wz = math.copysign(MAX_WZ * scale, err_o)
            wz = max(-MAX_WZ, min(MAX_WZ, wz))

        try:
            avoid_client.Move(vx, 0.0, wz)
        except Exception as e:
            print(f"[FOLLOW] Move error: {e}")

        time.sleep(DT)

    # On exit, stop motion
    try:
        avoid_client.Move(0.0, 0.0, 0.0)
    except Exception:
        pass


# ---------- Camera wrapper ----------
class Camera:
    """Interface to the robot's camera."""
    def __init__(self, timeout_sec: float = CAM_TIMEOUT_SEC):
        self.client = VideoClient()
        self.client.SetTimeout(timeout_sec)
        self.client.Init()

    def get_frame(self):
        """Return BGR frame as np.ndarray or None."""
        try:
            code, data = self.client.GetImageSample()
            if code == 0 and data:
                buf = np.frombuffer(bytes(data), dtype=np.uint8)
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is not None and img.size > 0:
                    return img
            return None
        except Exception as e:
            print(f"[CAM] Error getting image: {e}")
            return None

    def close(self):
        try:
            self.client.Close()
        except Exception:
            pass


# ---------- Global stop event & SIGINT handler ----------
stop_event = threading.Event()

def handle_sigint(signum, frame):
    """Handle Ctrl+C gracefully."""
    print("\n[SYS] Ctrl+C detected — stopping...")
    stop_event.set()


# ---------- Main ----------
def main():
    signal.signal(signal.SIGINT, handle_sigint)

    # Init UWB & subscribers
    state_manager = UwbStateManager()
    button_monitor = UwbButtonMonitor(state_manager, lambda: print("[UWB] Shutting down..."))
    ChannelFactoryInitialize(0)
    uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
    uwb_sub.Init(button_monitor.get_callback(), 10)

    # Init movement clients
    sport_client = SportClient()
    avoid_client = ObstaclesAvoidClient()
    avoid_client.Init()
    sport_client.Init()
    avoid_client.UseRemoteCommandFromApi(True)
    avoid_client.SwitchSet(True)

    # Small settle time for UWB
    time.sleep(1.0)
    try:
        print(f"[UWB] Initial Orientation: {state_manager.remote_state.orientation_est:.2f}")
    except Exception:
        print("[UWB] Initial orientation not available yet.")

    # Start follow loop in background
    t_follow = threading.Thread(
        target=follow_loop,
        args=(state_manager, avoid_client, stop_event),
        daemon=True
    )
    t_follow.start()
    print("[FOLLOW] Started background follow loop.")

    # Init camera (keep UI in main thread)
    cam = Camera(timeout_sec=CAM_TIMEOUT_SEC)
    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 960, 540)
    print("[CAM] Streaming... Press 'q' or Ctrl+C to exit.")

    # Camera/display loop in main thread
    last_t = time.time()
    frames = 0
    fps_t0 = last_t

    try:
        while not stop_event.is_set():
            frame = cam.get_frame()
            if frame is not None:
                cv2.imshow(WIN_NAME, frame)
                frames += 1

                # Lightweight FPS report every ~2s
                now = time.time()
                if now - fps_t0 >= 2.0:
                    fps = frames / (now - fps_t0)
                    print(f"[CAM] ~{fps:.1f} FPS")
                    fps_t0 = now
                    frames = 0

            # Allow GUI events and 'q' quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_event.set()
                break

            # Prevent busy loop if no frames
            if frame is None:
                time.sleep(0.01)

    finally:
        # Signal stop and cleanup
        stop_event.set()
        try:
            t_follow.join(timeout=1.5)
        except Exception:
            pass

        try:
            avoid_client.Move(0.0, 0.0, 0.0)
        except Exception:
            pass

        try:
            avoid_client.UseRemoteCommandFromApi(False)
            avoid_client.SwitchSet(False)
        except Exception:
            pass

        cam.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

        print("[SYS] Shutdown complete.")


if __name__ == "__main__":
    main()
