#!/bin/bash

if [ "$#" -eq 1 ]; then
  echo "Two args provided. Using folder $1"
  FOLDER=$1
elif [[ "$#" -eq 0 ]]; then
  echo "No args provided. Using actual folder"
  FOLDER=.
else
  echo "Invalid arguments"
  exit 1
fi

process() {
  local filename=$1
  pcl_outlier_removal "$filename.pcd" "/tmp/${filename}_or.pcd" -radius 0.2 -min_pts 30
  pcl_mls_smoothing "/tmp/${filename}_or.pcd" "output/${filename}_smooth.pcd" -radius 0.5
  pcl_voxel_grid "output/${filename}_smooth.pcd" "output/${filename}_vgf.pcd" -leaf 0.05,0.05,0.05
  pcl_outlier_removal "output/${filename}_vgf.pcd" "output/${filename}_vgf.pcd" -radius 0.2 -min_pts 10
  pcl_convert_pcd_ascii_binary "output/${filename}_vgf.pcd" "output/${filename}_vgf.pcd" 0
}

cd $FOLDER
mkdir -p output
for i in *.pcd; do
filename=${i%.*}
process "$filename" &
done
