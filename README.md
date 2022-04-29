This package is meant for developing custom plugins to facilitate for a bigger project.

The model_push_plugin is meant to simulate the fluid ejection force onto the UAV.

This model aims to subscribe to a known rostopic to allow the jet force to be turned on or off.

If you are installing this package onto a new PC for the first time, make sure to:
-> have gz_ros_plugin, PX4 autopilot, mavros, mavlink packages built/installed and working,
--> head over to PX4-Autopilot package and find the Tools>sitl_gazebo>models folder,
---> look for the iris folder (or whichever model folder that you will be using),
----> scroll to the bottom of the <model> segment and add
      <plugin name='model_push_plugin' filename="/home/your-pc-name/catkin_ws/devel/lib/libmodel_push_plugin.so"/>
      after the last plugin.
-----> roslaunch px4 posix_sitl.launch and you should see a force applied to the iris model
