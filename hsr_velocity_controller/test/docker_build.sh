#!/bin/bash
# Build the Docker image for testing
# This script should be run from the workspace root or will navigate there automatically

# Navigate to workspace root (3 levels up from test directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd "$WORKSPACE_ROOT" || exit 1

echo "Building Docker image from workspace root: $WORKSPACE_ROOT"
docker build -f src/iai_hsr/hsr_velocity_controller/test/Dockerfile -t hsr_controller_test .
