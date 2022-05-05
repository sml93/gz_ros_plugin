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


Date of update: 05 May 2022
Edited: jetForce topics to take in float mag and int angle to allow for negative angles, read /mavros/local_position/pose to track uav movement.
TODO: Check on jetForce angle rotation accuracy. Seems like negative angles pushes the uav back and vice versa. To create a counter to automatically send 0 jetForce mag after a certain time.
