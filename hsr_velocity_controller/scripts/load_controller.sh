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

# Source the workspace (robot: /home/custom_ws, remote PC: ~/custom_controller_ws)
for WS in /home/custom_ws "$HOME/custom_controller_ws"; do
    if [ -f "$WS/install/setup.bash" ]; then
        source "$WS/install/setup.bash"
        break
    fi
done

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
if ros2 control list_controllers 2>/dev/null | grep -q "$CONTROLLER_NAME"; then
    CURRENT_STATE=$(ros2 control list_controllers 2>/dev/null | grep "$CONTROLLER_NAME" | awk '{print $NF}' | tr -d '[]')
    echo "Controller '$CONTROLLER_NAME' already exists in state: $CURRENT_STATE"
    echo "Skipping load. To reload, first unload with:"
    echo "  ros2 service call ${CONTROLLER_MANAGER}/unload_controller controller_manager_msgs/srv/UnloadController \"{name: '$CONTROLLER_NAME'}\""
    exit 0
fi

# The Jazzy spawner has no --controller-type option, so set the type on the controller manager directly
ros2 param set $CONTROLLER_MANAGER ${CONTROLLER_NAME}.type $CONTROLLER_TYPE

# Load controller using spawner (loads, configures, but stays inactive)
# The param file is read by the controller manager, so this script must run on the robot
echo "Loading and configuring controller using spawner..."
ros2 run controller_manager spawner \
    $CONTROLLER_NAME \
    --controller-manager $CONTROLLER_MANAGER \
    --param-file $PARAM_FILE \
    --inactive

if [ $? -eq 0 ]; then
    echo "Controller loaded and configured successfully"
else
    echo "ERROR: Failed to load controller"
    exit 1
fi

echo ""
echo "=== Startup Complete ==="
echo "Controller '$CONTROLLER_NAME' is loaded and configured (inactive state)"
echo ""
echo "To switch to the velocity controller (also works from a remote PC):"
echo "  ./switch_controller.sh velocity"
echo ""
echo "To switch back to the trajectory controllers:"
echo "  ./switch_controller.sh trajectory"
echo ""
echo "To check status:"
echo "  ros2 control list_controllers"
