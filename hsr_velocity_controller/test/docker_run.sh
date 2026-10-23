#!/bin/bash
# Run the container with the workspace mounted and execute tests
# This script should be run from the workspace root or will navigate there automatically

# Navigate to workspace root (3 levels up from test directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd "$WORKSPACE_ROOT" || exit 1

echo "Running tests from workspace root: $WORKSPACE_ROOT"
docker run -it --rm \
  -v "$WORKSPACE_ROOT":/root/ws \
  hsr_controller_test \
  /bin/bash -c "source /opt/ros/humble/setup.bash && \
                colcon build --packages-select hsr_velocity_controller && \
                source install/setup.bash && \
                colcon test --packages-select hsr_velocity_controller --event-handlers console_direct+ && \
                colcon test-result --verbose || (cat build/hsr_velocity_controller/Testing/Temporary/LastTest.log && cat build/hsr_velocity_controller/test_results/hsr_velocity_controller/*.xml)"
