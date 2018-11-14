# ABB ROS Package

ABB ROS node by MCube Lab at MIT

## Table of contents

1. [Setting up the package](#setting-up-the-package)
2. [Operation modes](#operation-modes)
3. [Service calls](#service-calls)
4. [Available services](#available-services)
    1. [Standard services](#standard-services)
        1. [`Ping`](#ping)
        2. [`SetCartesianJ`](#setcartesianj)
    2. [EGM services](#egm-services)
        1. [`ActivateEGM`](#activateegm)
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

- **Matlab:** Check out the Matlab tutorial on [Creating a Service Client](https://www.mathworks.com/help/robotics/examples/call-and-provide-ros-services.html#d120e2265). Note that the Robotics System Toolbox should be installed.

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

#### `SetCartesianJ`

- `SetCartesianJ` moves the robot to a given cartesian position using a joint move. If we are currently in non-blocking mode, then we simply setup the move and let the non-blocking thread handle the actual moving. If we are in blocking mode, we then communicate with the robot to execute the move.

- **Input:** `float64 x`, `float64 y`, `float64 z`, `float64 q0`, `float64 qx`, `float64 qy`, `float64 qz`

- **Output:** `int64 ret`, `string msg`



### EGM services

#### `ActivateEGM`: EGM mode activations

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
