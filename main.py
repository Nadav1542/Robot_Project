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
        """Stop all movement."""
        print("Stopping all movement.")
        self.obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
        self.obstacles_avoid_client.UseRemoteCommandFromApi(False)

# Define the handler
def handle_sigint(signum, frame):
    print("\nSIGINT received (Ctrl+C). Cleaning up...")
    obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
    obstacles_avoid_client.UseRemoteCommandFromApi(False)
    sys.exit(0)  # Exit gracefully


if __name__ == "__main__":
    
    state_manager = UwbStateManager()
    button_monitor = UwbButtonMonitor(state_manager, lambda: print("[UWB] Shutting down..."))

    ChannelFactoryInitialize(0)
    uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
    uwb_sub.Init(button_monitor.get_callback(), 10)
    client = SportClient()
    obstacles_avoid_client = ObstaclesAvoidClient()
    
    # client.SetTimeout(10.0)
    # obstacles_avoid_client.SetTimeout(2.0)
    obstacles_avoid_client.Init()
    client.Init()
    
    obstacles_avoid_client.UseRemoteCommandFromApi(True)
    obstacles_avoid_client.SwitchSet(True)
    
    #signal handler for sig_init
    signal.signal(signal.SIGINT, handle_sigint)

    initial_orientation = state_manager.remote_state.orientation_est
    print(f"[UWB] Initial Orientation: {initial_orientation:.2f}")
    
        # --- tuning knobs ---
    TARGET = math.pi / 2                    # 90° in radians
    DEADBAND = math.radians(20)              # ±10° window where we don't move
    KP = 1.2                                # proportional gain -> how strongly to turn
    MAX_WZ = 1.0                            # cap on angular command sent to Move()
    DT = 0.02                               # 20 ms loop

    








    # t_end = time.time() + 150 # run for 30 seconds
    # while time.time() < t_end:
    #     current_orientation = state_manager.remote_state.orientation_est
    #     print(f"[UWB] Current Orientation: {current_orientation:.2f}")
    #     if current_orientation < TARGET:
    #         obstacles_avoid_client.Move(1, 0.0, 0.0)
    #         time.sleep(5)
    #     if current_orientation > TARGET:
    #         obstacles_avoid_client.Move(1, 0.0, 0.0)
    #         time.sleep(5)
    # #     # if TARGET - DEADBAND < current_orientation < TARGET + DEADBAND:
    # #     #     # inside the quiet zone → hold still
    # #     #     obstacles_avoid_client.Move(0.0, 0.0, 0.0)
    # #     #     print("stop move")
    # #     # else:
    # #     #     # outside → rotate toward the target
    # #     #     wz = (KP * current_orientation) % math.pi
    # #     #     # clamp to safe range
    # #     #     if wz >  MAX_WZ: wz =  MAX_WZ
    # #     #     if wz < -MAX_WZ: wz = -MAX_WZ
    # #     #     obstacles_avoid_client.Move(0.0, 0.0, wz)

        

    
    obstacles_avoid_client.Move(0, 0, 0)  # Stop obstacle avoidance movement
    obstacles_avoid_client.UseRemoteCommandFromApi(False)