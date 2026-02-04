#!/bin/bash

echo "Configuring can0..."
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 4000000 fd on
sudo ip link set can0 up
echo "can1 configured."

# 设置 can1 为 1M/4M CAN FD
echo "Configuring can1..."
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 4000000 fd on
sudo ip link set can1 up
echo "can1 configured."

# 显示状态
ip -details link show can0
ip -details link show can1
