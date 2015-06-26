#!/bin/bash
# common place for putting experiment commands
if [ -z "${1+set}" ]; then # argument 1 is case number
  echo "Please set argument 1 (command case number)!"
  exit -1
fi

data_path_prefix=E:  # for windows on office PC
# data_path_prefix=C:/Users/thanh  # for windows on Bootcamp Mac Pro
synthetic=1
# datatype="synthetic"
datatype="recording"
if [ -z "${3+set}" ]; then # argument 2 is data_path
  # if [ $synthetic -eq 1 ]; then
  "synthetic")
    prefix=$data_path_prefix/data/structural_modeling/synthetic_models
    data_path=$prefix/box1-1
    #data_path=$prefix/box2-1
    #data_path=$prefix/box2-2
    # data_path=$prefix/box3-1
    #data_path=$prefix/box3-2
    data_path=$prefix/box3-3
    data_path=$prefix/single-box-20150120
    data_path=$prefix/two-boxes-20150120
    # data_path=$prefix/desk1-20150203
    data_path=$prefix/desk2-20150203
    data_path=$prefix/livingroom1-20150203

    filename=blender_generated
    camera=" -c 640.0 -c 640.0 -c 320 -c 240 "
  ;;
  
  "recording")
    prefix=$data_path_prefix/data/structural_modeling/recording

    data_path=$prefix/20140612-box-20degree

    # data_path=$prefix/20140612-box-1 # good simple scene

    # data_path=$prefix/20140612-box-2 # good simple scene

    # data_path=$prefix/office-10 # good

    # data_path=$prefix/boxes-1 # good for segmentation
    # data_path=$prefix/boxes-2

    # # data_path=$prefix/kinect-white-boxes-1 # not good
    # data_path=$prefix/backup-1/kinect-white-boxes-2 # good slammap

    # data_path=$prefix/kitchen-1
    # data_path=$prefix/kitchen-2 # slammap quite ok

    # # results not clean enough
    # data_path=$prefix/kitchen-3 # good slammap

    # data_path=$prefix/corridor-4 # good

    # data_path=$prefix/seminar-3 # ok

    filename=kinect_recorder
    # kinect camera
    camera=" -c 529.21508098293293 -c 525.56393630057437 -c 328.94272028759258 -c 267.48068171871557 "
    # record_nframes=500 # stair case

  ;;
  esac
else
  data_path=$3
fi 

if [ ! -d $data_path ]; then
  echo "Given data_path($data_path) doesn't exist or not a directory!"
  exit -1;
fi

current_path="${PWD}"
echo "data_path:$data_path"
cd $data_path
echo "Case $1"


case "$1" in

"convert")
  convert_trajectory --rec $data_path/$filename.txt \
  --sseq $data_path/clams-sseq.bin --src $data_path/oslam.txt \
  --dst $data_path/clams-traj.bin

"gen")
  generate_map --sseq $data_path/clams-sseq.bin --traj $data_path/clams-traj.bin \
  --resolution 0.01 --max-range 2.0 --map $data_path/clams-map.pcd

"calibrate")
  calibrate --workspace $data_path --increment 1

"vis_mo")
  visualize_model --intrinsics $data_path/clams-calib.bin
  ;;

"vis_traj")
  visualize_trajectory --sseq $data_path/clams-sseq.bin --traj $data_path/clams-traj.bin \
  --map $data_path/clams-map.pcd
  ;;

esac


cd "$current_path"
