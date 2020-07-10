#!/bin/sh
MODEL=`rospack find offset_diff_drive_description`/urdf/robot.urdf.xacro
echo $MODEL | entr -r roslaunch urdf_tutorial display.launch model:=$MODEL
