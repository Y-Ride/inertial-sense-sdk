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

# Source the correct ros2 distro
if [ -z "$ROS_DISTRO" ]; then
  echo "Error: ROS_DISTRO environment variable is not set."
  exit 1
fi
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Build the ros2 packages
cd ../.. # Change to the ros2_ws
colcon build --packages-select InertialSenseSDK inertial_sense_ros2

# Add the source command to .bashrc file if it doen't aleady exist
# LINE="source $(pwd)/install/setup.bash"

# # Check if the line is already in ~/.bashrc
# if ! grep -Fxq "$LINE" ~/.bashrc; then
#   echo "$LINE" >> ~/.bashrc
#   echo "Added to ~/.bashrc: $LINE"
# else
#   echo "Line already exists in ~/.bashrc"
# fi

# Activate changes to .bashrc file
# echo "Sourcing the ~/.bashrc file"
# source ~/.bashrc

# Successfull completion!!! Oh ya!!!
echo -e "\nSetup complete!\n"
