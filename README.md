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
