ROS2's `mock_componennts` mimics the behaviour of robot in simulation without any physical robot.

Below is a miny tutorial for this setup.

# 1. What "Mock Hardware" does:

The `mock_components/GenericSystem` simulates joints interfaces defined in our URDF.

- It is a good choice to test controller logic and message flow.

# 2. Setup Overview:

*URDF (ros2_control.robot.xacro)

Inside `<ros2_control>` block, under `<hardware>` tag, ensure this selection exists:

```xml
    <ros2_control name="hsrb" type="system">
      <hardware>
        <plugin>mock_components/GenericSystem</plugin>
      </hardware>

      <!-- everything else as it is -->

    </ros2_control>
```
**Note: The joint names should match exactly what velocity controller expects (as seen in its log list)**

# 3. Controller config (my_controller_realtime_test.yaml)

Make sure this YAML defines your controller and its joints

# 4. Launch Mock test

We can reuse our `hsr.launch.py`

```bash
ros2 launch iai_hsr_bringup hsr.launch.py velocity_controller:=True
```

We will see something like this in logs:

```bash
[ros2_control_node-2] [INFO] [controller_manager]: Loaded hardware 'hsrb' from plugin 'mock_components/GenericSystem'
[ros2_control_node-2] [INFO] [realtime_body_controller_real]: Controller configured for 7 joints:
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - arm_flex_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - arm_lift_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - arm_roll_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - wrist_flex_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - wrist_roll_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - head_pan_joint
[ros2_control_node-2] [INFO] [realtime_body_controller_real]:   - head_tilt_joint
```

At this point, our system is running with *mock_hardware* 

# 5. send Commands to the controller

Our controller subscribes to `/command`(like `std_msgs/msg/Float64MultiArray`), test it using below command in another terminal:

```bash
ros2 topic pub /command std_msgs/msg/Float64MultiArray \
"data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

We can observe the movement of robot in `rviz2`.

# 6. Typical debug checks:

Issue

```
[spawner-3] [WARN] [spawner_realtime_body_controller_real]: Could not contact service /controller_manager/list_controllers
[spawner-3] [INFO] [spawner_realtime_body_controller_real]: waiting for service /controller_manager/list_controllers to become available...
```

Solution

Please make sure `/controller_manager` node is visible. If not, please check if `ros2_control` node is included in the launch file. 