#!/bin/bash
# HSR Controller Switch Script
# Switches the arm and head between the default trajectory controllers and the velocity controller.
# Only uses controller manager services, so it can be run on the robot or on a remote PC.
# The velocity controller is loaded first if needed.
#
# Usage: ./switch_controller.sh velocity|trajectory|status

set -e  # Exit on error

# Configuration
VELOCITY_CONTROLLER="realtime_body_controller_real"
VELOCITY_CONTROLLER_TYPE="hsr_velocity_controller_ns/HsrVelocityController"
TRAJECTORY_CONTROLLERS="arm_trajectory_controller head_trajectory_controller"
CONTROLLER_MANAGER="/controller_manager"
# The param file is read by the controller manager, so this must be its path on the robot
ROBOT_PARAM_FILE="${ROBOT_PARAM_FILE:-/home/custom_ws/install/hsr_velocity_controller/share/hsr_velocity_controller/config/realtime_body_controller.yaml}"

# Prints the state of a controller, or nothing if it is not loaded
controller_state() {
    ros2 control list_controllers -c $CONTROLLER_MANAGER 2>/dev/null | awk -v name="$1" '$1 == name {print $3}'
}

case "$1" in
    velocity)
        if [ -z "$(controller_state $VELOCITY_CONTROLLER)" ]; then
            echo "Loading '$VELOCITY_CONTROLLER'..."
            # Set type and param file on the controller manager directly instead of using the spawner,
            # which rejects param files that do not exist on the local machine
            ros2 service call $CONTROLLER_MANAGER/set_parameters rcl_interfaces/srv/SetParameters \
                "{parameters: [{name: '${VELOCITY_CONTROLLER}.type', value: {type: 4, string_value: '$VELOCITY_CONTROLLER_TYPE'}},
                               {name: '${VELOCITY_CONTROLLER}.params_file', value: {type: 9, string_array_value: ['$ROBOT_PARAM_FILE']}}]}" > /dev/null
            ros2 control load_controller $VELOCITY_CONTROLLER -c $CONTROLLER_MANAGER --set-state inactive
        fi
        DEACTIVATE=""
        for C in $TRAJECTORY_CONTROLLERS; do
            if [ "$(controller_state $C)" = "active" ]; then
                DEACTIVATE="$DEACTIVATE $C"
            fi
        done
        echo "Switching to velocity controller..."
        ros2 control switch_controllers -c $CONTROLLER_MANAGER --strict \
            ${DEACTIVATE:+--deactivate $DEACTIVATE} \
            --activate $VELOCITY_CONTROLLER
        ;;
    trajectory)
        # Trajectory controllers may have been unloaded (e.g. by the switch launch file), so reload them
        for C in $TRAJECTORY_CONTROLLERS; do
            if [ -z "$(controller_state $C)" ]; then
                echo "Loading '$C'..."
                ros2 run controller_manager spawner $C --controller-manager $CONTROLLER_MANAGER --inactive
            fi
        done
        DEACTIVATE=""
        if [ "$(controller_state $VELOCITY_CONTROLLER)" = "active" ]; then
            DEACTIVATE=$VELOCITY_CONTROLLER
        fi
        echo "Switching to trajectory controllers..."
        ros2 control switch_controllers -c $CONTROLLER_MANAGER --strict \
            ${DEACTIVATE:+--deactivate $DEACTIVATE} \
            --activate $TRAJECTORY_CONTROLLERS
        ;;
    status)
        ;;
    *)
        echo "Usage: $0 velocity|trajectory|status"
        exit 1
        ;;
esac

for C in $VELOCITY_CONTROLLER $TRAJECTORY_CONTROLLERS; do
    echo "  $C: $(controller_state $C || true)"
done
