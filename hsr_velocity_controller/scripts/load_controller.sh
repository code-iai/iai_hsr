#!/bin/bash
# HSR Velocity Controller Startup Script
# This script loads and configures the velocity controller at robot startup
# The controller remains in 'inactive' state until activated remotely

set -e  # Exit on error

# Configuration
CONTROLLER_NAME="realtime_body_controller_real"
CONTROLLER_TYPE="hsr_velocity_controller_ns/HsrVelocityController"
CONTROLLER_MANAGER="/controller_manager"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 and workspace
#source /opt/ros/humble/setup.bash
if [ -d "~/custom_controller_ws/install/setup.bash" ]; then
    source ~/custom_controller_ws/install/setup.bash
fi

# Get parameter file path
PARAM_FILE=$(ros2 pkg prefix hsr_velocity_controller)/share/hsr_velocity_controller/config/realtime_body_controller.yaml

echo "=== HSR Velocity Controller Startup ==="
echo "Controller: $CONTROLLER_NAME"
echo "Type: $CONTROLLER_TYPE"
echo "Param file: $PARAM_FILE"
echo ""

# Wait for controller manager to be ready
echo "Waiting for controller manager..."
MAX_WAIT=30
WAIT_COUNT=0
while ! ros2 service list | grep -q "${CONTROLLER_MANAGER}/list_controllers"; do
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $WAIT_COUNT -ge $MAX_WAIT ]; then
        echo "ERROR: Controller manager not ready after ${MAX_WAIT}s"
        exit 1
    fi
done
echo "Controller manager is ready"

# Check if controller already exists
#if ros2 control list_controllers 2>/dev/null | grep -q "$CONTROLLER_NAME"; then
#    echo "Controller '$CONTROLLER_NAME' already exists"
#    echo "  Use 'ros2 control set_controller_state $CONTROLLER_NAME inactive' to deactivate"
#    echo "  Or 'ros2 service call ${CONTROLLER_MANAGER}/unload_controller controller_manager_msgs/srv/UnloadController \"{name: '$CONTROLLER_NAME'}\"' to unload"
#    exit 0
#fi

# Set controller type parameter
echo "Setting controller type parameter..."
ros2 param set $CONTROLLER_MANAGER ${CONTROLLER_NAME}.type $CONTROLLER_TYPE
echo "Controller type set"

# Load parameters from file
echo "Loading controller parameters..."
ros2 param load $CONTROLLER_MANAGER $PARAM_FILE
echo "Parameters loaded"

# Load controller (this only loads, does not configure or activate)
echo "Loading controller..."
ros2 control load_controller $CONTROLLER_NAME
echo "Controller loaded"

# Configure controller (moves to 'inactive' state)
echo "Configuring controller..."
ros2 control set_controller_state $CONTROLLER_NAME configure
echo "Controller configured and ready"

echo ""
echo "=== Startup Complete ==="
echo "Controller '$CONTROLLER_NAME' is loaded and configured (inactive state)"
echo ""
echo "To activate remotely:"
echo "  ros2 control set_controller_state $CONTROLLER_NAME active"
echo ""
echo "To deactivate:"
echo "  ros2 control set_controller_state $CONTROLLER_NAME inactive"
echo ""
echo "To check status:"
echo "  ros2 control list_controllers"
