# ROS2_STM32_Comm

A ROS2 Node for sending commands to and receiving data from an STM32 running the associated firmware

## Description

This node is intended to be modified to add more commands later or change all the commands, SerialComm sends raw commands while StmComm is a wrapper to have nice functions to use

## Getting Started

### Dependencies

- ROS2 Humble
- [libserial](https://github.com/crayzeewulf/libserial)  

### Installing

This repo needs the custom messages of [ROS2_Custom_Msg](https://github.com/The-Last-Resort-FR/ROS2_Custom_Msg)
Clone the repositories
```bash
git clone https://github.com/The-Last-Resort-FR/ROS2_STM32_Comm.git
git clone https://github.com/The-Last-Resort-FR/ROS2_Custom_Msg.git
```  
Build ROS2_Custom_Msg first then ROS2_STM32_Comm
```bash
cd ROS2_Custom_Msg
colcon build
source install/setup.sh
cd ../ROS2_STM32_Comm
colcon build --symlink-install
```
Source the environement of ROS2_STM32_Comm
```bash
source install/setup.bash
```

### Executing program

Run the node  
```bash
ros2 launch stm_comm launch.py
```

## TODO

- Seperate wrapper library to handle communication with this node
- Resend

## Tested Hardware

- Jetson AGX Orin
- STM32 Nucleo F411RE 

## Version History

- V0.0.0 : Raw comm and structure
- V0.1.0 : ROS2 Server and cleanup

## License

This project is licensed under the "The Unlicense" License - see the LICENSE.md file for details