MuJoCo ROS 2 Control Plugins
=============================

The ``mujoco_ros2_control_plugins`` package provides a plugin interface for extending the
functionality of ``mujoco_ros2_control``.
This separation allows for modular, optional features without adding complexity to the core package.

.. note::

   This interface provides flexibility for accessing information from the MuJoCo model and data.
   Users are responsible for handling that data correctly and avoiding changes to critical information.


Available Plugins
-----------------

HeartbeatPublisherPlugin
~~~~~~~~~~~~~~~~~~~~~~~~

A simple demonstration plugin that publishes a heartbeat message every second to the
``/mujoco_heartbeat`` topic.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Topic**
     - ``mujoco_heartbeat`` (``std_msgs/String``)
   * - **Rate**
     - 1 Hz
   * - **Message format**
     - ``"MuJoCo ROS2 Control Heartbeat #N | Simulation time: Xs"``

**Example: monitoring the heartbeat**

.. code-block:: bash

   # Terminal 1: launch your mujoco_ros2_control simulation
   ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py

   # Terminal 2: echo the heartbeat messages
   ros2 topic echo /mujoco_heartbeat


SimulationManagementPlugin
~~~~~~~~~~~~~~~~~~~~~~~~~~

Provides services for inspecting and managing the live MuJoCo simulation. The plugin does not
perform periodic work; its ``update()`` implementation is empty.

.. list-table::
   :widths: 25 75
   :header-rows: 0

   * - **Service**
     - ``~/get_robot_state`` (``mujoco_ros2_control_msgs/srv/GetRobotState``)
   * - **Service**
     - ``~/set_obstacle`` (``mujoco_ros2_control_msgs/srv/SetObstacle``)
   * - **Service**
     - ``~/throw_food`` (``mujoco_ros2_control_msgs/srv/ThrowFood``)

The state-service request is empty. Its response contains ``float64[] qpos`` with all ``model->nq``
generalized position values, plus the managed box's ``obstacle_position`` and full
``obstacle_size``. Size axes are width (X), length (Y), and height (Z).

``set_obstacle`` accepts the desired XY position and the same full size vector. It edits the named
``obstacle`` geom in the retained ``mjSpec`` and invokes ``mj_recompile`` while holding the
simulation mutex. The requested Z position is ignored; the geom centre is always placed at half
its height so its bottom face remains on the floor.

``throw_food`` teleports a free-floating food body into one of the numbered parking areas. The
request carries the target ``parking_index`` (1..``parking_count``), the ``food_name`` of the body
to move, an ``x``/``y`` position expressed in that parking's frame, and a 4-element ``orientation``
quaternion in MuJoCo order (``w, x, y, z``). The position and orientation are composed with the
parking body's current world pose; the world-frame drop height is taken from the
``throw_food_height`` parameter (the requested Z is fixed). The handler rewrites the body's
free-joint ``qpos`` and zeroes its ``qvel`` while holding the simulation mutex, so the item starts
at rest. The orientation is normalised, so an un-normalised quaternion is accepted.

**Example**

.. code-block:: bash

   ros2 service call /simulation_management/get_robot_state \
     mujoco_ros2_control_msgs/srv/GetRobotState "{}"

   ros2 service call /simulation_management/set_obstacle \
     mujoco_ros2_control_msgs/srv/SetObstacle \
     "{position: {x: 1.0, y: -2.0}, size: {x: 0.8, y: 1.2, z: 1.0}}"

   ros2 service call /simulation_management/throw_food \
     mujoco_ros2_control_msgs/srv/ThrowFood \
     "{parking_index: 1, food_name: 'food_box', x: 0.25, y: 0.0, orientation: [1.0, 0.0, 0.0, 0.0]}"

**Example configuration**

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         simulation_management:
           type: "mujoco_ros2_control_plugins/SimulationManagementPlugin"
           throw_food_height: 0.3
           parking_count: 4


Usage
-----

Plugins are loaded from ROS 2 parameters under ``mujoco_plugins``.
Each plugin entry requires:

- A unique key (e.g. ``heart_beat_plugin``)
- A ``type`` field with the pluginlib class name

.. code-block:: yaml

   /**:
     ros__parameters:
       mujoco_plugins:
         heart_beat_plugin:
           type: "mujoco_ros2_control_plugins/HeartbeatPublisherPlugin"
           update_rate: 1.0

Pass this file to the ``mujoco_ros2_control`` node via ``ParameterFile(...)`` in your launch file.

.. note::

   In this repository, ``mujoco_ros2_control_demos/launch/01_basic_robot.launch.py`` already loads
   ``mujoco_ros2_control_demos/config/mujoco_ros2_control_plugins.yaml``.


Creating Your Own Plugin
------------------------

1. Create the Plugin Header
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a header that inherits from ``MuJoCoROS2ControlPluginBase``:

.. code-block:: cpp

   #include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

   namespace my_namespace
   {

   class MyCustomPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
   {
   public:
     bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
     void update(const mjModel* model, mjData* data) override;
     void cleanup() override;

   private:
     // Your member variables
   };

   }  // namespace my_namespace

2. Implement the Plugin Methods
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cpp

   #include "my_custom_plugin.hpp"
   #include <pluginlib/class_list_macros.hpp>

   namespace my_namespace
   {

   bool MyCustomPlugin::init(
     rclcpp::Node::SharedPtr node,
     const mjModel* model,
     mjData* data)
   {
     // Initialize your plugin
     return true;
   }

   void MyCustomPlugin::update(const mjModel* model, mjData* data)
   {
     // Called every control loop iteration
   }

   void MyCustomPlugin::cleanup()
   {
     // Clean up resources
   }

   }  // namespace my_namespace

   PLUGINLIB_EXPORT_CLASS(
     my_namespace::MyCustomPlugin,
     mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
   )

3. Create the Plugin XML Descriptor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create ``my_plugins.xml``:

.. code-block:: xml

   <library path="my_plugin_library">
     <class name="my_namespace/MyCustomPlugin"
            type="my_namespace::MyCustomPlugin"
            base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase">
       <description>
         Description of what your plugin does.
       </description>
     </class>
   </library>

4. Update CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: cmake

   find_package(mujoco_ros2_control_plugins REQUIRED)

   add_library(my_plugin_library SHARED
     src/my_custom_plugin.cpp
   )

   target_link_libraries(my_plugin_library
     ${mujoco_ros2_control_plugins_TARGETS}
     # ... other dependencies
   )

   pluginlib_export_plugin_description_file(
     mujoco_ros2_control_plugins
     my_plugins.xml
   )


Plugin Lifecycle
----------------

1. **Initialization** (``init``): Called once when the plugin is loaded. Use this to read
   parameters and set up publishers, subscribers, and services.
2. **Update** (``update``): Called every simulation step at the **end of the** ``read`` **loop**,
   before the controller update and ``write`` loops. Changes to ``mjData`` here are visible to
   controllers and affect the next simulation step. This runs in a real-time thread — avoid
   blocking operations.
3. **Cleanup** (``cleanup``): Called when shutting down. Release any resources acquired in
   ``init``.

Service, subscription, or worker callbacks that directly access the live ``mjModel`` or ``mjData``
passed to ``init()`` must hold the simulation's recursive mutex. The loader sets this mutex before
``init()`` and derived plugins can access it through the protected ``simulation_mutex()`` method.


Building
--------

This package is part of the ``mujoco_ros2_control`` workspace:

.. code-block:: bash

   colcon build --packages-select mujoco_ros2_control_plugins
