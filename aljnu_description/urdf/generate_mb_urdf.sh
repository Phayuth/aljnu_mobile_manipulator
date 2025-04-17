#!/bin/bash

# Define variables
DESCRIPTION_PACKAGE="aljnu_description"
DESCRIPTION_FILE="aljnu_mp.urdf.xacro"
OUTPUT_FILE="aljnu_mp.urdf"

# Find the xacro executable
XACRO_EXEC=$(which xacro)
if [ -z "$XACRO_EXEC" ]; then
    echo "Error: xacro is not installed or not in PATH."
    exit 1
fi

# Generate the URDF file
$XACRO_EXEC "$(ros2 pkg prefix $DESCRIPTION_PACKAGE)/share/$DESCRIPTION_PACKAGE/urdf/$DESCRIPTION_FILE" \
    safety_limits:=true \
    safety_pos_margin:=0.15 \
    safety_k_position:=20 \
    name:=ur \
    ur_type:=ur5e \
    tf_prefix:=ur5e_ > $OUTPUT_FILE

if [ $? -eq 0 ]; then
    echo "URDF file generated successfully: $OUTPUT_FILE"
else
    echo "Error: Failed to generate URDF file."
    exit 1
fi