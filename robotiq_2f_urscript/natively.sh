#!/bin/bash

# Environment configuration
export GRIPPER_ID="g1"
export TEMPLATES_DIR="/home/endre/r2r_ws/src/robotiq_2f/robotiq_2f_urscript/templates"

export UR_ADDRESS="192.168.1.31"
export UR_PORT="30002"

echo "Environment variables set:"
echo "GRIPPER_ID=${GRIPPER_ID}"
echo "TEMPLATES_DIR=${TEMPLATES_DIR}"
echo "UR_ADDRESS=${UR_ADDRESS}"
echo "UR_PORT=${UR_PORT}"