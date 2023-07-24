#!/bin/bash

# Assign the filename
# source .env

filename="urdf.urdf"

# Take the search string
# read -p "Enter the search string: " search

# # Take the replace string
# read -p "Enter the replace string: " replace
search="rom"
replace="$(whoami)"


if [[ $search != "" && $replace != "" ]]; then
  sed -r "s/$search/$replace/" $filename
fi