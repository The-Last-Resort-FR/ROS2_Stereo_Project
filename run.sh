#!/bin/bash

# Exit immediately on error
set -e

source install/setup.sh

sudo chmod 777 /dev/ttyACM1

cleanup() {
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null
    wait
    echo "All processes terminated."
}

trap cleanup SIGINT

ros2 run stm_comm stm_comm > stm_comm.log 2>&1 &
ros2 launch camera_manager launch.py > camera_manager.log 2>&1 &
ros2 run image_processor image_processor > image_processor.log 2>&1 &

wait
