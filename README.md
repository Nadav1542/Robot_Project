Quick Start

Environment Dependencies
System environment

It is recommended to use the Ubuntu 20.04 system for development. Development on Mac and Windows systems is currently not supported, and development on the built-in computer of Go2 is also not supported.
Network environment

It is necessary to set the network card in the user's computer that communicates with the Go2 robot under the 123 network segment, and the recommended IP address of the network card is 192.168.123.222 ("222" can be changed to other).
It is not allowed to set the IP address of the network card to 192.168.123.161, which is the built-in computer IP address of the Go2 robot.

Configure network environment

When running the routine, control commands will be sent from the user's computer to the built-in computer of the Go2 robot through the local area network.
Therefore, before this, necessary configuration steps need to be taken to form a local area network between these two computers.

Configuration steps:

1. Connect one end of the network cable to the Go2 robot, and the other end to the user's computer.
2. Turn on the USB Ethernet of the computer and configure it.
The IP address of the onboard computer of the machine dog is 192.168.123.161, so it is necessary to set the USB Ethernet address of the computer to the same network segment as the machine dog.
For example, entering 192.168.123.222 ("222" can be changed to other) in the Address field.
3. Change the 'netmask' field to 255.255.255.0

example:
https://doc-cdn.unitree.com/static/2023/9/6/0f51cb9b12f94f0cb75070d05118c00a_980x816.jpg

This setup is required for communication with the robot.

To test whether the user's computer is properly connected to the built-in computer of the Go2 robot, you can enter ping 192.168.123.161 in the terminal for testing.
The connection is successful when something similar to the following appears.

https://doc-cdn.unitree.com/static/2023/8/31/393207d38e3a49cda738a418036ae8f0_1088x438.png



