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


def stop(obstacles_avoid_client):
        """Stop all movement."""
        print("Stopping all movement.")
        obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
        obstacles_avoid_client.UseRemoteCommandFromApi(False)

# Define the handler
def handle_sigint(signum, frame):
    print("\nSIGINT received (Ctrl+C). Cleaning up...")
    obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
    obstacles_avoid_client.UseRemoteCommandFromApi(False)
    sys.exit(0)  # Exit gracefully

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

def follow_me():
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
                        break
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

def print_data(time_sec = 60):
    
    t_end = time.time() + time_sec
    
    while time.time() < t_end:
        dis = state_manager.remote_state.distance_est
        ori = state_manager.remote_state.orientation_est
        yaw = state_manager.remote_state.yaw_est
        tag_yaw = state_manager.remote_state.tag_yaw
        print(f"\033[94m[UWB] Distance: {dis:.2f} m | Orientation: {math.degrees(ori):.2f}° | Yaw Est: {math.degrees(yaw):.2f}° | Tag Yaw: {math.degrees(tag_yaw):.2f}°\033[0m")
        
        time.sleep(1)

def initialize_system():
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

    #signal handler for sig_init
    signal.signal(signal.SIGINT, handle_sigint)
    return state_manager, button_monitor, client, obstacles_avoid_client


if __name__ == "__main__":
    
    state_manager, button_monitor, client, obstacles_avoid_client = initialize_system()

   
    
    t_end = time.time() + 150
    while time.time() < t_end:
        dis = state_manager.remote_state.distance_est
        err_d = dis - 1 
                
        if  -0.2 < err_d < 0.2:
              obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
              print("Stopped")
        elif err_d < -0.2:
                print("Walking")
                walk_me()
        else:
                print("Following")
                follow_me()
        
    
    stop(obstacles_avoid_client)