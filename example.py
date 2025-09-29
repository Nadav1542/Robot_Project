import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient, PathPoint

def main():
    # 1) Initialize DDS factory (must come first)
    ChannelFactoryInitialize(0)     # 0 = default domain id

   # 2) Now it's safe to create RPC clients
    client = SportClient()
       # client.SetTimeout(5.0)

   # 3) Build 30 PathPoints
    path = []
    for i in range(30):
            # Each point spaced in time by 0.1s
            t = 0.1 * (i + 1)

            # Example path: move forward in X, no Y, keep yaw 0
            p = PathPoint(timeFromStart=t, x=0.1 * (i + 1), y=0.0, yaw=0.0, vx=0.1, vy=0.0, vyaw=0.0)
           

            path.append(p)

        # 4) Send trajectory to robot
    ret = client.TrajectoryFollow(path)
    print("OK" if ret == 0 else f"Failed with code {ret}")

    time.sleep(3)
    

if __name__ == "__main__":
    main()
