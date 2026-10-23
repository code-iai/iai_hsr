#!/bin/bash
# Verify that the hsr_velocity_controller plugin is properly registered with pluginlib
# This runs the verification tool in the Docker container
# This script should be run from the workspace root or will navigate there automatically

# Navigate to workspace root (3 levels up from test directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd "$WORKSPACE_ROOT" || exit 1

echo "Running plugin verification from workspace root: $WORKSPACE_ROOT"
docker run -it --rm \
  -v "$WORKSPACE_ROOT":/root/ws \
  hsr_controller_test \
  /bin/bash -c "source /opt/ros/humble/setup.bash && \
                cd /root/ws && \
                colcon build --packages-select hsr_velocity_controller && \
                source install/setup.bash && \
                ros2 run hsr_velocity_controller verify_plugin_registration"
