#!/bin/bash


find_urdf="/home/rom"
replace_urdf="/home/$(whoami)"
param_old="/home/$(whoami)/ros2/ros2_ws/install/rom2109_description"
parma_new="$(ros2 pkg prefix rom2109_description)"


function find_replace_in_file {
    local file="$1"
    sed -i "s#${find_urdf}#${replace_urdf}#g; s#${param_old}#${parma_new}#g" "$file"
}


find_replace_in_file "urdf.urdf"

