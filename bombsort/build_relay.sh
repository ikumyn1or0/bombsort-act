#!/bin/bash
cd /workspace/bombsort && g++ -O2 pose_relay.cc -o /home/ubuntu/pose_relay $(pkg-config --cflags --libs ignition-transport11) && echo BUILD_OK
echo "bomb_red 0.2 0.04 1.037" | timeout 5 /home/ubuntu/pose_relay
