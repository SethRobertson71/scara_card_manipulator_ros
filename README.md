# scara_card_manipulator_ros
ROS node source code for SCARA card manipulator robot project for SAU Engineering Robotics class, 2026, implemented as a docker container.

# WSL Camera with usbipd
If using a docker contained in WSL, you must run `usbipd attach --wsl --busid <BUSID>`. BUSID can be found by runnning `usbipd --list`.