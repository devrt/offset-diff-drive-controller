#!/bin/sh
MODEL=`rospack find offset_diff_drive_description`/urdf/robot.urdf.xacro
echo $MODEL | entr -r roslaunch offset_diff_drive_description empty_world.launch model:=$MODEL
