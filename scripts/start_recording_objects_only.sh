#!/bin/bash

#Otherwise rosbag does not work.
source $HOME/.bashrc

#Make sure to write files directly on disk and not on nfs share.
cd /data_ext/data/gehrung/

# Remove old file, if exists
rm OBJ_${1}.bag

#Record raw images of stereo setup as well as corresponding calibration information as this is the most compressed non stream data representation available. Record object localization results as well as their 3D visualizations as well to have both raw data and recognizer results in a synchronized manner. Save tf information for data to be able to play back without additional nodes publishing tf frames with wrong timestamps.
rosbag record -b 1000 -O OBJ_${1} /stereo/visualization_marker /stereo/objects /tf


