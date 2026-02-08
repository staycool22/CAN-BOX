#!/bin/bash

# Setup script for HPM ETH-CAN 4-Channel Bridge
# Requires: 'cannelloni' installed and in PATH
# Requires: sudo privileges for network config

REMOTE_IP="192.168.1.10"
BASE_PORT=20000

# 1. Load kernel module
echo "Loading vcan module..."
sudo modprobe vcan

# 2. Setup vcan0 - vcan3
for i in {0..3}; do
    IFACE="vcan$i"
    if ! ip link show $IFACE > /dev/null 2>&1; then
        echo "Creating $IFACE..."
        sudo ip link add dev $IFACE type vcan
    fi
    sudo ip link set up $IFACE
    echo "$IFACE is UP"
done

# 3. Stop existing instances
echo "Stopping existing cannelloni processes..."
sudo killall cannelloni 2>/dev/null

# 4. Start 4 instances of cannelloni
echo "Starting cannelloni bridge instances..."

for i in {0..3}; do
    PORT=$((BASE_PORT + i))
    IFACE="vcan$i"
    
    # -I: CAN Interface
    # -R: Remote IP (HPM Board)
    # -r: Remote Port
    # -l: Local Port
    # -S: Sort by sequence number (optional, good for UDP)
    # -C: Data compression (optional, verify if HPM supports it? No, firmware implementation is raw)
    # Note: Do NOT use -C or encryption if firmware doesn't support it.
    
    CMD="cannelloni -I $IFACE -R $REMOTE_IP -r $PORT -l $PORT"
    echo "Running: $CMD"
    nohup $CMD > /dev/null 2>&1 &
    PID=$!
    echo "Started bridge: $IFACE <-> $REMOTE_IP:$PORT (PID: $PID)"
done

echo "Done. 4 Channels ready."
echo "Use 'TZETHCANTransmitter' in your Python script to control baud rate and send data."
