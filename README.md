# Unitree Go2 Development Quick Start

This guide provides the necessary steps to set up your environment, configure the network, install the SDK, and test communication with the **Unitree Go2** robot.

-----

## 💻 1. Environment Dependencies

Ensure your system meets these requirements before proceeding.

### Recommended Environment

  * **Operating System:** Use **Ubuntu 20.04** for development.

### Unsupported Environments

  * Development on **Mac** and **Windows** is **not supported**.
  * Development directly on the **Go2 robot's built-in computer** is **not supported**.

-----

## 🌐 2. Network Configuration

You must create a Local Area Network (LAN) between your computer and the robot to send control commands.

### Network Requirements

| Setting | Value/Requirement | Notes |
| :--- | :--- | :--- |
| **Robot IP** | **192.168.123.161** | This is the fixed IP of the Go2 robot's built-in computer. |
| **Computer IP** | **192.168.123.x** | Must be in the **192.168.123.x** segment. **192.168.123.222** is recommended. |
| **Netmask** | **255.255.255.0** | |

### Configuration Steps (USB Ethernet)

1.  **Physical Connection:** Connect one end of a network cable to the **Go2 robot** and the other end to your computer's **USB Ethernet** port.
2.  **Access Settings:** Turn on and configure the **USB Ethernet** interface on your computer.
3.  **Set IP Address:** Manually set a valid IP address, such as **192.168.123.222**, in the **Address** field.
4.  **Set Netmask:** Change the **'netmask'** field to **255.255.255.0**.

-----

## ✅ 3. Test the Connection

Verify that your computer can communicate with the robot.

1.  **Open Terminal:** Open a terminal window on your computer.
2.  **Run Ping Test:** Ping the robot's built-in computer:
    ```bash
    ping 192.168.123.161
    ```
3.  **Confirm Success:** The connection is successful if you receive replies in the terminal output.

----


  -------------------create venv------------------





## 🚀 4. Install and Test the SDK

Once the network is confirmed, install the official SDK and test basic control.

### SDK Installation

Install the Python SDK from the official Unitree repository (refer to the repository instructions for the full installation guide):

  * **Official Repository:** Unitree SDK2 Python

### Test with an Example

Run a provided example to confirm the SDK works correctly before moving forward.

1.  **Locate Example:** Find the `go2_sport_client.py` example in the `unitree_sdk2_python/example/go2/high_level` directory.
2.  **Run Example:** Execute the example script following the repository instructions.
3.  **Provide Input:** When prompted for an **ID**, enter **3**.
4.  **Verify Action:** The robot should respond by taking **one step forward**.

### Clone Your Project Repository

Finally, clone your specific project repository into your workspace.

1.  **Clone Repository:** In your desired workspace, clone the project. It's recommended to name the directory **GO2** for clarity.
    ```bash
    git clone https://github.com/Nadav1542/Robot_Project.git GO2
    ```
2.  **Navigate:** Change into the newly created directory to begin development.
    ```bash
    cd GO2
    ```

  --------run the project with XXXXXX------------




  Code Flow Overview

The robot operates by tracking a UWB remote carried by the user. When it detects an object of a specified type, it switches to an approach mode toward that object, and once the interaction is complete, it returns to user-following mode.

At any given moment, there are two threads running in parallel:

One thread is responsible for tracking the user.

The other handles object classification in the incoming frames.

The frames are provided by a separate class called Camera, and the classification is performed using an existing YOLO model.

The coordination between the two threads is managed through a shared variable named behavior, which determines the robot’s active state.


UWB Follow Algorithm

The FollowController class implements the robot’s UWB-based walking logic, responsible for continuously adjusting its position and orientation relative to the UWB remote carried by the user.

Algorithm Overview

The controller runs in a background thread and repeatedly performs the following steps:

Read UWB Measurements
The robot receives two main values from the UWB system:

distance_est — estimated distance to the remote.

orientation_est — estimated angle of the remote relative to the robot’s heading (ranging from -π to π).

Distance Control (vx_follow)
The robot computes a forward velocity based on the distance error (err_d).

If the distance is within a deadband (DEAD_BAND_D), the robot stops moving forward.

Otherwise, it moves toward or away from the remote, with velocity scaled proportionally to the distance (up to MAX_VX_FOLLOW).

The scaling ensures smooth acceleration and deceleration using the parameter DIST_SLOWDOWN.

Orientation Control (wz_follow)
Similarly, the robot adjusts its rotational velocity based on the orientation error (err_o).

When the angle is within a deadband (DEAD_BAND_O), no rotation occurs.

For larger deviations, the angular velocity is scaled by SLOWDOWN_ANGLE and limited by MAX_WZ_FOLLOW.

Behavior-Based Blending
The final movement command depends on the robot’s current behavior mode, stored in the shared behavior dictionary:

In "FOLLOW" mode — the robot uses the computed UWB-based velocities (vx_follow, wz_follow).

In "APPROACH" or "HOLD" mode — it instead uses externally provided target velocities (behavior["vx"], behavior["wz"]).

Command Execution
The resulting velocity commands are sent to the motion controller (avoid_client.Move(vx, 0, wz)), and the process repeats at a fixed control rate (FOLLOW_DT, typically 25 Hz).

Safe Stop
When the stop event is triggered, the controller sends a zero-velocity command to safely halt the robot.

Key Features

Threaded Execution – allows continuous following without blocking other robot behaviors.

Smooth Motion – uses proportional scaling and deadbands to reduce jitter and noise effects.

Behavior Integration – seamlessly switches between following and object-approach logic through the shared behavior variable.
