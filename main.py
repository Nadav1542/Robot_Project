import time
import threading
import signal
import sys
import math
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor



def stop(self):
    print("Stopping all movement.")
    self.obstacles_avoid_client.Move(0, 0, 0)
    self.obstacles_avoid_client.UseRemoteCommandFromApi(False)


# signal handler for killing
def handle_sigint(signum, frame):
    print("\nSIGINT received (Ctrl+C). Cleaning up...")
    obstacles_avoid_client.Move(0, 0, 0)
    obstacles_avoid_client.UseRemoteCommandFromApi(False)
    sys.exit(0)

"""
rotation by angle of remote bearing from robot head
acceleration spining dependes on error
"""
def rotation_by_yaw(self):
    # --- Adaptive rotation settings ---
    MAX_WZ = 0.96                # hard speed limit (rad/s)
    DT = 0.04                   # control loop (s)
    DEAD_BAND = 0.35           # stop when within ±0.35 rad (~20°)
    SLOWDOWN_ANGLE = math.radians(60)  # start tapering below 60°
    TARGET_YAW_RIGHT = math.pi              # target yaw (rad)
    TARGET_YAW_LEFT = -math.pi              # target yaw (rad)
    
    t_end = time.time() + 150
    while time.time() < t_end:
        yaw = state_manager.remote_state.yaw_est

        # Target is 0 rad. Compute shortest signed error and apply a simple,
        # piecewise-linear speed map that tapers near the target.
        # yaw - 0
        if yaw < 0:
            err = yaw - TARGET_YAW_LEFT
        else:
            err = yaw - TARGET_YAW_RIGHT

        if abs(err) <= DEAD_BAND:
            wz = 0.0
        else:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err) / SLOWDOWN_ANGLE, 1.0)
            wz = math.copysign(MAX_WZ * scale, err)

        # Safety clamp (keeps within [-0.9, 0.9] even if constants change)
        wz = max(-MAX_WZ, min(MAX_WZ, wz))

        obstacles_avoid_client.Move(0.0, 0.0, wz)
        time.sleep(DT)


"""
rotation by angle of remote location from robot head
acceleration spining dependes on error
"""
def rotation_by_orientation(self):
    # --- Adaptive rotation settings ---
    MAX_WZ = 0.96                # hard speed limit (rad/s)
    DT = 0.04                   # control loop (s)
    DEAD_BAND = 0.20            # stop when within ±0.20 rad (~11.5°)
    SLOWDOWN_ANGLE = math.radians(60)  # start tapering below 60°

    t_end = time.time() + 150
    while time.time() < t_end:
        ori = state_manager.remote_state.orientation_est

        # Target is 0 rad. Compute shortest signed error and apply a simple,
        # piecewise-linear speed map that tapers near the target.
        # ori - 0
        err = ori

        if abs(err) <= DEAD_BAND:
            wz = 0.0
        else:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err) / SLOWDOWN_ANGLE, 1.0)
            wz = math.copysign(MAX_WZ * scale, err)

        # Safety clamp (keeps within [-0.9, 0.9] even if constants change)
        wz = max(-MAX_WZ, min(MAX_WZ, wz))

        obstacles_avoid_client.Move(0.0, 0.0, wz)
        time.sleep(DT)

"""
robot moves to the user circle of radius 1.2 around the user
moves while trying to align its head to the remote
using distance and orientation

TODO:
- trying to bring the robot the spot where it should start walk me
- removing looping for 150 seconds
"""
def handle_follow():
      # --- Adaptive distance settings ---
    MAX_VX = 0.9               # hard speed limit (m/s)
    DT = 0.04                   # control loop (s)
    DEAD_BAND_D = 1.2            # stop when within ±1.2 m
    SLOWDOWN_ANGLE = math.radians(60)  # start tapering below 60°
    DEAD_BAND_O = 0.20            # stop when within ±0.20 rad (~11.5°)
    MAX_WZ = 0.96                # hard speed limit (rad/s)
    DIST_SLOWDOWN  = 1.0         # start tapering below 1.0 m


    t_end = time.time() + 150
    while time.time() < t_end:
        dis = state_manager.remote_state.distance_est
        ori = state_manager.remote_state.orientation_est
        
        # Target is 0 m. Compute shortest signed error and apply a simple,
        # piecewise-linear speed map that tapers near the target.
        # dis - 0
        err_d = dis  
        err_o = ori 

        if abs(err_d) <= DEAD_BAND_D:
            vx = 0.0
        else:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err_d) / DIST_SLOWDOWN, 1.0)
            vx = math.copysign(MAX_VX * scale, err_d)
        # Safety clamp (keeps within [-0.9, 0.9] even if constants change)
        vx = max(-MAX_VX, min(MAX_VX, vx))
        
        
        if abs(err_o) <= DEAD_BAND_O:
            wz = 0.0
        else:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err_o) / SLOWDOWN_ANGLE, 1.0)
            wz = math.copysign(MAX_WZ * scale, err_o)

        # Safety clamp (keeps within [-0.9, 0.9] even if constants change)
        wz = max(-MAX_WZ, min(MAX_WZ, wz))

        obstacles_avoid_client.Move(vx, 0.0, wz)
        time.sleep(DT)

"""
when robot iniside user circle it always moves forward (vx >=0)
and direction depends on the user bearing (which is perpendicular to remote heading)
when robot outside the user circle it stops moving
using distance and yaw

TODO:
- removing looping for 60 seconds
- checking how inital yaw (when turning on remote) affects the movement
- maybe adding orientation to have more stable movement 
"""
def walk_me():
       # --- Adaptive rotation settings ---
    MAX_WZ = 0.96                # hard speed limit (rad/s)
    DT = 0.04                   # control loop (s)
    DEAD_BAND_Y = 0.35           # stop when within ±0.35 rad (~20°)
    SLOWDOWN_ANGLE = math.radians(60)  # start tapering below 60°
    TARGET_YAW_RIGHT = math.pi              # target yaw (rad)
    TARGET_YAW_LEFT = -math.pi              # target yaw (rad)

      # --- Adaptive distance settings ---
    MAX_VX = 0.9               # hard speed limit (m/s)
    DT = 0.04                   # control loop (s)
    DEAD_BAND_D = 1.2            # stop when within ±1.2 m
    DIST_SLOWDOWN  = 1.0         # start tapering below 1.0 m


    t_end = time.time() + 60 # run for 60 seconds
    prev = time.time()
    while time.time() < t_end:
        dis = state_manager.remote_state.distance_est
        yaw = state_manager.remote_state.yaw_est

        if yaw < 0:
            err_y = yaw - TARGET_YAW_LEFT
        else:
            err_y = yaw - TARGET_YAW_RIGHT
        err_d = dis
        
        if abs(err_y) < DEAD_BAND_Y:
            wz = 0.0
        else:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err_y) / SLOWDOWN_ANGLE, 1.0)
            wz = math.copysign(MAX_WZ * scale, err_y)
            wz = max(min(wz, MAX_WZ), -MAX_WZ)  # enforce hard limit
        if abs(err_d) < DEAD_BAND_D:
            # scale ∈ (0,1]: large error → scale≈1 (fast), small error → scale→0 (slow)
            scale = min(abs(err_d) / DIST_SLOWDOWN, 1.0)
            vx = math.copysign(MAX_VX * scale, err_d)
            vx = max(min(vx, MAX_VX), -MAX_VX)  # enforce hard limit
        else:
            vx = 0.0
            break
        obstacles_avoid_client.Move(vx, 0, wz)
        time.sleep(DT)


if __name__ == "__main__":
    state_manager = UwbStateManager()
    button_monitor = UwbButtonMonitor(state_manager, lambda: print("[UWB] Shutting down..."))

    ChannelFactoryInitialize(0)
    uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
    uwb_sub.Init(button_monitor.get_callback(), 10)
    client = SportClient()
    obstacles_avoid_client = ObstaclesAvoidClient()

    obstacles_avoid_client.Init()
    client.Init()
    obstacles_avoid_client.UseRemoteCommandFromApi(True)
    obstacles_avoid_client.SwitchSet(True)

    signal.signal(signal.SIGINT, handle_sigint)

    print(f"[UWB] Initial Orientation: {state_manager.remote_state.orientation_est:.2f}")
    
    

    

    

   
    

    obstacles_avoid_client.Move(0, 0, 0)
    obstacles_avoid_client.UseRemoteCommandFromApi(False)

