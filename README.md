This document outlines the **Quick Start** instructions for setting up your development environment to communicate with the **Unitree Go2** robot. This involves system environment recommendations and crucial network configuration steps.

-----

## 💻 System Environment Dependencies

To ensure a smooth development process, adhere to the following system requirements:

  * **Recommended OS:** Use **Ubuntu 20.04** for development.
  * **Unsupported OS/Environments:**
      * Development on **Mac** and **Windows** systems is **not currently supported**.
      * Development directly on the **built-in computer of the Go2 robot** is **not supported**.

-----

## 🌐 Network Environment Configuration

Communication between your computer and the Go2 robot's built-in computer occurs over a local area network (LAN). Therefore, you must configure your computer's network interface to be on the same network segment as the robot.

### Network Requirements

  * **Network Segment:** Your computer's network card must be set within the **192.168.123.x** (the "123 network segment").
  * **Recommended IP:** The recommended IP address for your computer's network card is **192.168.123.222** (the last octet, "222", can be changed to any valid number in the segment).
  * **Prohibited IP:** Do **not** set your computer's IP address to **192.168.123.161**, as this is the fixed IP address of the Go2 robot's built-in computer.

### Configuration Steps (USB Ethernet)

Follow these steps to establish the required LAN connection:

1.  **Physical Connection:** Connect one end of a network cable to the **Go2 robot** and the other end to your **user's computer**.
2.  **Access Network Settings:** Turn on and configure the **USB Ethernet** interface on your computer.
3.  **Set IP Address:** Set the USB Ethernet's address to the same network segment as the robot (which has the IP **192.168.123.161**).
      * Enter a valid IP address, such as **192.168.123.222**, in the **Address** field. (Remember, you can change the last number).
4.  **Set Netmask:** Change the **'netmask'** field to **255.255.255.0**.

-----

## ✅ Testing the Connection

After configuring the network, you should verify the connection between your computer and the Go2 robot.

1.  **Open Terminal:** Open a terminal window on your computer.
2.  **Run Ping Test:** Enter the following command to ping the robot's built-in computer:
    ```bash
    ping 192.168.123.161
    ```
3.  **Successful Connection:** The connection is successful if you receive a reply, which will show something similar to: .
