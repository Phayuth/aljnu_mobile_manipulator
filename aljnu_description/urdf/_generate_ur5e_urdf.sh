#!/bin/bash

ros2 run xacro xacro ur5e/ur.urdf.xacro \
    safety_limits:=true \
    safety_pos_margin:=0.15 \
    safety_k_position:=20 \
    name:=ur \
    ur_type:=ur5e \
    tf_prefix:=ur5e_ > ur5e.urdf