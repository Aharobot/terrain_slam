#!/bin/bash
for i in *.ply; do
filename=${i%.*}
pcl_ply2pcd "$filename.ply" "$filename.pcd"
pcl_convert_pcd_ascii_binary "$filename.pcd" "$filename.pcd" 0
done
