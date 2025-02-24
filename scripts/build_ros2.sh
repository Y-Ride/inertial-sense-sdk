#!/bin/bash

# Ensure the script runs with root permissions
if [ "$(id -u)" -ne 0 ]; then
  echo "This script must be run as root." 1>&2
  exit 1
fi

# Get the original directory to go back at the end
ORIGINAL_DIR="$(pwd)"

# Get the directory where the script is located
SCRIPT_DIR=$(dirname "$(realpath "$0")")
echo ${SCRIPT_DIR}

# Change to the Inertial Sense SDK directory
cd "${SCRIPT_DIR}"/.. || { echo "Failed to change directory to $(pwd "${SCRIPT_DIR}/..")"; exit 1; }

# Get the parent directory
PARENT_DIR=$(basename "$(dirname "$(pwd)")")

# Check if the parent directory is "src"
if [ "$PARENT_DIR" != "src" ]; then
  echo "Error: The parent directory is not named 'src'. It is named '$PARENT_DIR'." 1>&2
  exit 1
fi

# Install sdk dependencies
set -e
./scripts/install_sdk_dependencies.sh
echo "sdk dependencies are successfully installed"

# Check if the symbolic link already exists
if [ -L "../ros2" ]; then
  echo "Symbolic link '${PARENT_DIR}/ros2' already exists. Removing it..."
  sudo rm "../ros2"
fi

# Create the symbolic link
sudo ln -s "$(pwd)/ROS/ros2" "../ros2"
echo "Create symbolic link for ros2 directory"

# Build the ros2 packages
cd ../.. # Change to the ros2_ws
set -e
colcon build --packages-select InertialSenseSDK inertial_sense_ros2

echo -e "\nSetup complete!\n"
