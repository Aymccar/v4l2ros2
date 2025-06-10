#!/bin/bash
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback   devices=2   video_nr=42,43   card_label="VirtualCam42,VirtualCam43"   exclusive_caps=1,1

