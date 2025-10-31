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
On startup the HSR shoulda always start with is default controllers active.
Use the launchfile `switch_to_velocity_controllers.launch.py` to deactivate those and activate the velocity controller.

> **Note:** Before launching `switch_to_velocity_controllers.launch.py`, ensure that the `/controller_manager` node is already running.
