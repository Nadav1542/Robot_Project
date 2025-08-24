import time
import threading
import signal
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import UwbState_
from uwb_state_manager import UwbStateManager
from uwb_button_monitor import UwbButtonMonitor

if __name__ == "__main__":

    state_manager = UwbStateManager()
    button_monitor = UwbButtonMonitor(state_manager, lambda: print("[UWB] Shutting down..."))

    ChannelFactoryInitialize(0)
    uwb_sub = ChannelSubscriber("rt/uwbstate", UwbState_)
    uwb_sub.Init(button_monitor.get_callback(), 10)
    initial_yaw = state_manager.remote_state.yaw_est
    print(f"[UWB] Initial Yaw: {initial_yaw:.2f}")
    while True:
        state = state_manager.remote_state
        current_yaw = state.yaw_est
        current_distance = state.distance_est
        # print(f"[UWB] Current Yaw: {current_yaw:.2f}")
        print(f"[UWB] Current Distance: {current_distance:.2f}")

    time.sleep(1)