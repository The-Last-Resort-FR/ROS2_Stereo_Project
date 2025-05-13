#!/bin/bash

colcon build --packages-select custom_msg
source install/setup.sh
colcon build