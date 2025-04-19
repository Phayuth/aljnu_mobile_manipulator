# AIRLAB JNU Mobile Manipulator ROS2
This package is written for the Robotic Manipulator project in my lab in ROS2. [For Mom!]

# What to config first ?
- Calibrate UR5e kinematic using ur_calibration, obtain the .yaml file, and place it [Here](./aljnu_description/config/ur5e/default_kinematics.yaml)
- Change UR5e to Scout2 transformation [Here](./aljnu_description/urdf/ur5e/ur.urdf.xacro#L98) and link name [Here](./aljnu_description/urdf/ur5e/ur.urdf.xacro#L64)

## [Read more here!](./_doc/)