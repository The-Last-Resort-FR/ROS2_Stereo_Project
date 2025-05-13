#!/bin/bash

sudo cp misc/stm_comm.service           /etc/systemd/system/
sudo cp misc/camera_manager.service     /etc/systemd/system/
sudo cp misc/image_processing.service   /etc/systemd/system/

[ -d /opt/stereo-project ] || sudo mkdir /opt/stereo-project

sudo cp -r install /opt/stereo-project

sudo systemctl daemon-reload