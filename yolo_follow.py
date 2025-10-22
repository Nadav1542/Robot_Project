# Comments in English only
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from go2_follow_app import Go2FollowApp

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    Go2FollowApp().run()
