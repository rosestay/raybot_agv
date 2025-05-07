#!/bin/bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can2 down

sudo ip link del can0
sudo ip link del can1
sudo ip link del can2

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can2 type can bitrate 1000000

sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2

