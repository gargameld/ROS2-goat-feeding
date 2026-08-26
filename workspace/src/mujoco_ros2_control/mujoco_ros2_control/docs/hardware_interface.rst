Hardware Interface Configuration
=================================

Plugin
------

The MuJoCo hardware interface is shipped as a ``ros2_control`` plugin.
Specify it in your URDF and point to a valid MJCF on launch:

.. code-block:: xml

   <ros2_control name="MujocoSystem" type="system">
     <hardware>
       <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>


       <!-- Optional: camera RGB-D image publish rate in Hz (all cameras share one rate).
            Defaults to 5 Hz. -->
       <param name="camera_publish_rate">6.0</param>
     </hardware>
   ...

Due to compatibility requirements, a slightly modified ``ros2_control`` node is required.
It is the same executable and accepts the same parameters as the upstream node:

.. code-block:: python

   control_node = Node(
       # Use the node from this package
       package="mujoco_ros2_control",
       executable="ros2_control_node",
       output="both",
       parameters=[
           {"use_sim_time": True},
           controller_parameters,
       ],
   )

.. note::

   The custom node can be removed after the next ``ros2_control`` upstream release, which will include
   the required changes.

Joints
------

Joints in the ``ros2_control`` interface are mapped by name to actuators defined in the MJCF.
The system supports different joint control modes based on the actuator type and available command interfaces.

MuJoCo's PD-level ``ctrl`` input is used for direct position, velocity, or effort control, so each command
interface requires a natively matching MuJoCo actuator type.
Incompatible actuator-interface combinations trigger an error at startup.

Refer to MuJoCo's `actuation model <https://mujoco.readthedocs.io/en/stable/computation/index.html#geactuation>`_ for more information.

Only one type of MuJoCo actuator per-joint can be controllable at a time, and the type **cannot** be switched at runtime.
However, the active command interface can be switched dynamically, allowing control to shift between position, velocity, or effort as supported by the actuator type.

For example, a position-controlled joint in MJCF:

.. code-block:: xml

   <actuator>
     <position joint="joint1" name="joint1" kp="25000" dampratio="1.0" ctrlrange="0.0 2.0"/>
   </actuator>

Maps to the following ``ros2_control`` hardware interface:

.. code-block:: xml

   <joint name="joint1">
     <command_interface name="position"/>
     <!-- Initial values for state interfaces default to 0 if not specified -->
     <state_interface name="position">
       <param name="initial_value">0.0</param>
     </state_interface>
     <state_interface name="velocity"/>
     <state_interface name="effort"/>
   </joint>

**Supported modes between MuJoCo actuators and ros2_control command interfaces:**

.. list-table::
   :header-rows: 1
   :stub-columns: 1

   * - Command Interface
     - MuJoCo ``position``
     - MuJoCo ``velocity``
     - MuJoCo ``motor``, ``general``, etc.
   * - **position**
     - Native support
     - Not supported
     - Not supported
   * - **velocity**
     - Not supported
     - Native support
     - Not supported
   * - **effort**
     - Not supported
     - Not supported
     - Native support

.. note::

   The ``torque`` and ``force`` command/state interfaces are semantically equivalent to ``effort`` and map to the same underlying data in the sim.

Grippers
--------

Many robot grippers drive several joints from a single actuator.
The hardware interface does not implement ``ros2_control`` mimic joints; couple the joints inside MuJoCo
instead, and expose only the driving joint to ``ros2_control``.

For parallel jaw mechanisms, we recommend combining tendon actuators with an equality constraint.
For example, from the test robot:

.. code-block:: xml

   <actuator>
     <position tendon="split" name="gripper_left_finger_joint" kp="1000" dampratio="3.0" ctrlrange="-0.09 0.005"/>
   </actuator>
   <tendon>
     <fixed name="split">
       <joint joint="gripper_left_finger_joint" coef="0.5"/>
       <joint joint="gripper_right_finger_joint" coef="-0.5"/>
     </fixed>
   </tendon>
   <equality>
     <joint joint1="gripper_left_finger_joint" joint2="gripper_right_finger_joint" polycoef="0 -1 0 0 0" solimp="0.95 0.99 0.001" solref="0.005 1"/>
   </equality>

The tendon name matches the controllable joint in the ``ros2_control`` configuration.
The drivers expose control and state for that single joint, while the simulation enforces the coupling internally.

Sensors
-------

The hardware interface supports inertial measurement units (IMUs).
MuJoCo does not model a complete IMU natively, so we combine supported MJCF sensor constructs to map to a single ``ros2_control`` sensor.

IMU
~~~

Simulate a ``framequat``, ``gyro``, and ``accelerometer`` as a single IMU:

.. code-block:: xml

   <sensor>
     <framequat name="imu_sensor_quat" objtype="site" objname="imu_sensor"/>
     <gyro name="imu_sensor_gyro" site="imu_sensor"/>
     <accelerometer name="imu_sensor_accel" site="imu_sensor"/>
   </sensor>

Map to the corresponding ``ros2_control`` sensor:

.. code-block:: xml

   <sensor name="imu_sensor">
     <param name="mujoco_type">imu</param>
     <!-- mujoco_sensor_name does not need to match the ros2_control sensor name.
          The MJCF sensors are looked up as <mujoco_sensor_name> plus the fixed
          suffixes _quat, _gyro and _accel. -->
     <param name="mujoco_sensor_name">imu_sensor</param>
     <state_interface name="orientation.x"/>
     <state_interface name="orientation.y"/>
     <state_interface name="orientation.z"/>
     <state_interface name="orientation.w"/>
     <state_interface name="angular_velocity.x"/>
     <state_interface name="angular_velocity.y"/>
     <state_interface name="angular_velocity.z"/>
     <state_interface name="linear_acceleration.x"/>
     <state_interface name="linear_acceleration.y"/>
     <state_interface name="linear_acceleration.z"/>
   </sensor>

These sensor state interfaces work out of the box with the standard ROS 2 broadcasters.

Cameras
-------

Any ``camera`` included in the MJCF will automatically have an organized point cloud published to a ROS topic.

The camera ``name`` attribute sets the defaults for the frame and topic names:

- Frame: ``<name>_frame``
- Topic: ``<name>/points``

For example:

.. code-block:: xml

   <camera name="wrist_mounted_camera" fovy="58" mode="fixed" resolution="640 480" pos="0 0 0" quat="0 0 0 1"/>

Publishes the following topics:

.. code-block:: bash

   $ ros2 topic info /wrist_mounted_camera/points
   Type: sensor_msgs/msg/PointCloud2

Frame and topic names can be overridden via ``ros2_control`` xacro:

.. code-block:: xml

   <!-- The sensor name must match the camera name in the MJCF -->
   <sensor name="wrist_mounted_camera">
     <param name="frame_name">wrist_mounted_camera_mujoco_frame</param>
     <param name="pointcloud_topic">/wrist_mounted_camera/points</param>
   </sensor>

.. note::

   MuJoCo's camera coordinate conventions differ from ROS. Refer to the MuJoCo documentation for details.

Simulation Topics
=================

``/clock`` (``rosgraph_msgs/msg/Clock``)
   Contains the internal physics clock tracked by each MuJoCo simulation step.
