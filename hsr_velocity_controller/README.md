## Package Description
This package contains the source code for a ros_control controller that is meant to controll the joints of the Toyota HSR via velocity commands.
As the HSR hardware accepts position commands this controller achieves this by combining a feedworward term that integrates the desired velocity together with a PID output of the difference between measured and desired joint velocity.

## Installation Instructions
To use the launch file you have to have this package installed in the ros workspace of your external computer.
To have the velocity controller also working on the real HSR do the following steps:
1. ssh into the hsr
2. Create an overlay workspace and copy the hsr_velocity_controller package into the src folder of the workspace
3. Build the workspace
4. Activate the overlay workspace in the HSR config files (A deatiled description can be found on hsr.io)
5. After restarting the HSR the command `ros2 control list_controllers` should also list the hsr_velocity_controller

## Usage Instructions
On startup the HSR should always start with its default controllers active.
Use `scripts/switch_controller.sh` to switch between them and the velocity controller. It only calls controller manager services, so it can be run on the robot or on a remote PC:
```bash
./scripts/switch_controller.sh velocity    # loads the velocity controller if needed and activates it
./scripts/switch_controller.sh trajectory  # switches back to the arm and head trajectory controllers
./scripts/switch_controller.sh status
```
The controller manager reads the param file on the robot, so `switch_controller.sh` passes the robot-side path (`/home/custom_ws/install/...`). Set `ROBOT_PARAM_FILE` if the workspace on the robot is located elsewhere.

`scripts/load_controller.sh` only loads the velocity controller in the inactive state and must be run on the robot.

Velocity commands are sent as `std_msgs/msg/Float64MultiArray` in the order of the configured joints. The controller has no command timeout, so send zeros to stop:
```bash
ros2 topic pub --once /realtime_body_controller_real/command std_msgs/msg/Float64MultiArray "{data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

> **Note:** The launch file `switch_to_velocity_controllers.launch.py` unloads the trajectory controllers and passes a local param file path, so it only works when run on the robot.
