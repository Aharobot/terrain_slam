#!/bin/bash
cd ../output
rm *.pcd
for i in *.ply; do
filename=${i%.*}
pcl_ply2pcd "$filename.ply" "$filename.pcd"
done
pcl_viewer *.pcd