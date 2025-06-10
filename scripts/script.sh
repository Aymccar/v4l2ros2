#!/bin/bash
gst-launch-1.0 v4l2src device=/dev/video42 !     video/x-raw,format=I420,width=1920,height=1080,framerate=30/1 !     x264enc key-int-max=1 tune=zerolatency bitrate=10000 speed-preset=faster !     h264parse config-interval=-1 !     video/x-h264,stream-format=byte-stream,alignment=au !     v4l2sink device=/dev/video43


