# ABB ROS Package

ABB ROS node by MCube Lab at MIT

## Table of contents

1. [Setting up the package](#setting-up-the-package)
2. [Operation modes](#operation-modes)
3. [Service calls](#service-calls)
4. [Available services](#available-services)
    1. [Standard services](#standard-services)
        1. [`Ping`](#ping)
        2. [`SetCartesian`](#setcartesian)
        3. [`SetCartesianJ`](#setcartesianj)
        4. [`GetCartesian`](#getcartesian)
        5. [`SetJoints`](#setjoints)
        6. [`GetJoints`](#getjoints)
        7. [`GetIK`](#getik)
        8. [`GetFK`](#getfk)
        9. [`Stop`](#stop)
        10. [`SetTool`](#settool)
        11. [`SetInertia`](#setinertia)
        12. [`SetWorkObject`](#setworkobject)
        13. [`SetComm`](#setcomm)
        14. [`SetMotionSupervision`](#setmotionsupervision)
        15. [`SetSpeed`](#setspeed)
        16. [`SetAcc`](#setacc)
        17. [`GetState`](#getstate)
        18. [`SetZone`](#setzone)
        19. [`SetTrackDist`](#settrackdist)
        20. [`SetDefaults`](#setdefaults)
        21. [`IsMoving`](#ismoving)
        22. [`AddBuffer`](#addbuffer)
        23. [`ExecuteBuffer`](#executebuffer)
        24. [`ClearBuffer`](#clearbuffer)
        25. [`AddJointPosBuffer`](#addjointposbuffer)
        26. [`ExecuteJointPosBuffer`](#executejointposbuffer)
        27. [`ClearJointPosBuffer`](#clearjointposbuffer)
        28. [`ActivateCSS`](#activatecss)
        29. [`DeactivateCSS`](#deactivatecss)
        30. [`IOSignal`](#iosignal)
    2. [EGM services](#egm-services)
        1. [`ActivateEGM`](#activateegm)
        2. [`StopEGM`](#stopegm)
5. [Examples](#examples)

## Setting up the package
1. Clone this repo under the `catkin_ws` folder of your project.
2. Use `catkin_make` to compile all packages and dependencies.
3. Update the parameter file in `robot_node/parameters/mcubeRobotParams.yaml` with the proper robot values.
4. Update the launch file in `robot_node/launch/mcubeSystem.launch` with the proper robot identifier.
5. Start running the RAPID robot server pressing *Play* in the FlexPendant.
6. Finally, launch `roslaunch robot_node mcubeSystem.launch`.

If connection with the ABB controller is successful, a variety of ROS services will now be available. You can check them by typing `rosservice call` and then pressing Tab.

## Operation modes

- **Standard blocking mode:** User can operate the robot via [standard services](#standard-services). Function calls (e.g. movements) are blocking until the operation is about to finish and cannot be stopped.

- **Standard non-blocking mode:** User can operate the robot via [standard services](#standard-services). Instructions are not sent immediately to the robot; intermediate targets are sent instead. Therefore, user can stop or change final destinations before final target is achieved. Function calls are non-blocking.

- **EGM mode:** User can operate the robot pubishing target poses or velocities to the [`SetCartesian`] topic, depending on the chosen EGM mode when calling [`ActivateEGM`](#activateegm). Current pose and joint state can be obtained via `GetCartesian` and `GetJoints` topics, respectively.

- **RRI mode:** EGM predecessor, deprecated.

When started, the ROS node is working in standard and blocking mode. Standard non-blocking mode can be activated using the [`SetComm`](#setcomm) service, and EGM can be activated via the [`ActivateEGM`](#activateegm) service.

## Service calls

Note that, for any interface, the specific service name is given by the robot identifier and the service type itself. For example, the `Ping` service for `robot1` will be `/robot1_Ping`.

- **Command line**: Any of the services listed below can be called via the `rosservice call` command via terminal. For example, to ping a robot identified by `robot1`, call `rosservice call /robot1_Ping`.

- **Python:** Check out the ROS tutorial on [Writing a Simple Service Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_the_Client_Node).

- **C++:** Check out the ROS tutorial on [Writing a Simple Service Client (C++)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29#roscpp_tutorials.2BAC8-Tutorials.2BAC8-WritingServiceClient.Writing_the_Client_Node).

- **Matlab:** Check out the Matlab tutorial on [Creating a Service Client](https://www.mathworks.com/help/robotics/examples/call-and-provide-ros-services.html#d120e2265). Note that Robotics System Toolbox is required.

  Furthermore, as these services use custom messages for requests and returns, the [Robotics System Toolbox Interface for ROS Custom Messages](https://www.mathworks.com/help/robotics/ug/install-robotics-system-toolbox-support-packages.html) should be installed as well, in order to generate the custom messages for Matlab. After installed, use the `rosgenmsg` to generate them and follow the instructions. For example:

  ```
  rosgenmsg('/home/mcube/example_project/catkin_ws/src/abb-ros-catkin/')
  ```

## Available services

### Standard services

#### `Ping`

- `Ping` sends a dummy message to the robot and waits for its answer, in order to check the connection.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `SetCartesian`

- `SetCartesian` moves the robot to a given cartesian position.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`

#### `SetCartesianJ`

- `SetCartesianJ` moves the robot to a given cartesian position using a joint move.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`

#### `GetCartesian`

- `GetCartesian` queries the cartesian position of the robot.

- **Input:** empty

- **Output:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`, `int64 ret`, `string msg`

#### `SetJoints`

- `SetJoints` moves the robot to a given joint position.

- **Input:** `float64 j1`, `float64 j2`, `float64 j3`, `float64 j4`, `float64 j5`, `float64 j6`

- **Output:** `int64 ret`, `string msg`

#### `GetJoints`

- `GetJoints` queries the robot for the current position of its joints.

- **Input:** empty

- **Output:** `float64 j1`, `float64 j2`, `float64 j3`, `float64 j4`, `float64 j5`, `float64 j6`, `int64 ret`, `string msg`

#### `GetIK`

- `GetIK` queries the robot for the inverse kinematics (joint angles) given a cartesian position.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `float64 j1`, `float64 j2`, `float64 j3`, `float64 j4`, `float64 j5`, `float64 j6`, `int64 ret`, `string msg`

#### `GetFK`

- `GetFK` queries the robot for the forward kinematics (cartesian position) given the joint angles.

- **Input:** `float64 j1`, `float64 j2`, `float64 j3`, `float64 j4`, `float64 j5`, `float64 j6`

- **Output:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`, `int64 ret`, `string msg`

#### `Stop`

- `Stop` stops the robot when operating in standard non-blocking mode.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `SetTool`

- `SetTool` sets the tool frame of the robot.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`

#### `SetInertia`

- `SetInertia` sets the inertia of the tool of the robot.

- **Input:** `float64 m`, `float64 cgx`, `float64 cgy`, `float64 cgz`, `float64 ix`, `float64 iy`, `float64 iz`

- **Output:** `int64 ret`, `string msg`

#### `SetWorkObject`

- `SetWorkObject` sets the work object of the robot.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`

#### `SetComm`

- `SetComm` sets the communication mode of our robot (standard blocking mode or standard non-blocking mode).

- **Input:** `int64 mode` (0 for non-blocking, 1 for blocking)

- **Output:** `int64 ret`, `string msg`

#### `SetMotionSupervision`

- `SetMotionSupervision` sets the motion supervision of the robot.

- **Input:** `float64 supervision` (between 1 and 300, recommended: 50)

- **Output:** `int64 ret`, `string msg`

#### `SetSpeed`

- `SetSpeed` sets the speed of the robot in standard mode. This will affect the step size in non-blocking mode or the actual speed in blocking mode.

- **Input:** `float64 tcp` (in mm/s), `float64 ori` (in deg/s)

- **Output:** `int64 ret`, `string msg`

#### `SetAcc`

- `SetAcc` sets the acceleration of the robot in standard mode. This will affect the step size in non-blocking mode or the actual speed in blocking mode.

- **Input:** `float64 acc` (in mm/s^2), `float64 deacc` (in mm/s^2)

- **Output:** `int64 ret`, `string msg`


#### `GetState`

- `GetState` gets the current state of the robot.

- **Input:** empty

- **Output:** `float64 tcp` (in mm/s), `float64 ori` (in deg/s), `int64 zone`, `int64 vacuum`, `float64 workx`, `float64 worky`, `float64 workz`, `float64 workq0`, `float64 workqx`, `float64 workqy`, `float64 workqz`, `float64 toolx`, `float64 tooly`, `float64 toolz`, `float64 toolq0`, `float64 toolqx`, `float64 toolqy`, `float64 toolqz`, `float64 toolm`, `float64 toolcgx`, `float64 toolcgy`, `float64 toolcgz`, `float64 toolix`, `float64 tooliy`, `float64 tooliz`, `int64 ret`, `string msg`

#### `SetZone`

- `SetZone` sets the zone of the robot. This is the distance before the end of a motion that the server will respond. This enables smooth motions.

- Read the service definition at `robot_comm/srv/robot_SetZone.srv` for more details.

#### `SetTrackDist`

- `SetTrackDist` sets the tracking distance of the robot while in standard non-blocking mode.

- **Input:** `float64 pos_dist` (in mm), `float64 ang_dist` (in deg)

- **Output:** `int64 ret`, `string msg`

#### `SetDefaults`

- `SetDefaults` restores the robot to default configuration.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `IsMoving`

- `IsMoving` returns whether the robot is moving or not. In blocking mode, this will always return false, as the robot will move only when a function is being called (and is blocking).

- **Input:** empty

- **Output:** `bool moving`, `int64 ret`, `string msg`

#### `AddBuffer`

- `AddBuffer` adds a TCP pose buffer command.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`

#### `ExecuteBuffer`

- `ExecuteBuffer` executes the buffer defined by several `AddBuffer` service calls.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `ClearBuffer`

- `ClearBuffer` clears the buffer defined by several `AddBuffer` service calls.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `AddJointPosBuffer`

- `AddJointPosBuffer` adds a joint position buffer command.

- **Input:** `float64 j1`, `float64 j2`, `float64 j3`, `float64 j4`, `float64 j5`, `float64 j6`

- **Output:** `int64 ret`, `string msg`

#### `ExecuteJointPosBuffer`

- `ExecuteBuffer` executes the buffer defined by several `AddJointPosBuffer` service calls.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `ClearJointPosBuffer`

- `ClearBuffer` clears the buffer defined by several `AddJointPosBuffer` service calls.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

#### `ActivateCSS`

- `ActivateCSS` activates Cartesian Soft Servo.

- **Input:** `int32 refFrame`, `float64 refOrient_q0`, `float64 refOrient_qx`, `float64 refOrient_qy`, `float64 refOrient_qz`, `int32 softDir`, `float64 stiffness`, `float64 stiffnessNonSoftDir`, `int32 allowMove`, `float64 ramp`

- **Output:** `int64 ret`, `string msg`

#### `DeactivateCSS`

- `DeactivateCSS` deactivates Cartesian Soft Servo.

- **Input:** `geometry_msgs/Pose ToPose`

- **Output:** `int64 ret`, `string msg`

#### `IOSignal`

- `IOSignal` sets I/O signals to the robot.

- **Input:** `int32 output_num` (from 1 to 4), `int32 signal` (0 for off, 1 for on)

- **Output:** `int64 ret`, `string msg`


### EGM services

#### `ActivateEGM`

- `ActivateEGM` activates EGM. If called when standard non-blocking mode is running, all movements will be stopped. After calling it, all other services will be disabled except `StopEGM` until this last one is called or timeout.

- **Input:** `bool mode` (`0` for pose mode, `1` for velocity mode), `int64 timeout` (in seconds)

- **Output:** `int64 ret`, `string msg`

#### `StopEGM`

- `StopEGM` stops EGM. After calling it, all other services will be enabled again and standard mode will be recovered.

- **Input:** empty

- **Output:** `int64 ret`, `string msg`

## Examples

- Run a series of joint configurations:

  ```
  rosservice call /robot1_ClearJointPosBuffer
  rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 90 0
  rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 91 0
  rosservice call -- /robot1_AddJointPosBuffer 0 0 0 0 89 0
  rosservice call /robot1_ExecuteJointPosBuffer
  ```

- Run a series of cartesian configurations:

  ```
  rosservice call /robot1_ClearBuffer
  rosservice call /robot1_SetSpeed 50 50 # apply to the following knot points until the next set speed.
  rosservice call -- /robot1_AddBuffer 300 0 300 1 0 0 0    # x y z (mm) q0 qx qy qz
  rosservice call -- /robot1_AddBuffer 300 0 301 1 0 0 0
  rosservice call /robot1_SetSpeed 50 100
  rosservice call -- /robot1_AddBuffer 300 0 300 1 0 0 0
  rosservice call /robot1_ExecuteBuffer  # go through the whole trajectory
  ```
  Note: Too small spacing between points may cause jerky motions. Try SetZone to higher value.

- Set 24V IO signals:

  ```
  rosservice call /robot1_IOSignal [output_num] [signal]  # output_num = 1,2,3,4; #signal 1:on 0:off
  rosservice call /robot1_IOSignal 1 1  # signal output channel 1 to on
  rosservice call /robot1_IOSignal 1 0  # signal output channel 1 to off
  ```
