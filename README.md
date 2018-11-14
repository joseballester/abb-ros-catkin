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

## Service calls

### Command line

Any of the services listed below can be called via the `rosservice call` command, followed by the specific service name given by the robot identifier and the service type itself.

For example, to ping a robot idenitifed by `robot1`:

```
rosservice call /robot1_Ping
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
