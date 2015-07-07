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
datatype="calib"
if [ -z "${3+set}" ]; then # argument 2 is data_path
  case "$datatype" in
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

    # data_path=$prefix/20140612-box-20degree

    # data_path=$prefix/20140612-box-1 # good simple scene

    # data_path=$prefix/20140612-box-2 # good simple scene

    data_path=$prefix/office-10 # good

    # data_path=$prefix/boxes-1 # good for segmentation
    # data_path=$prefix/boxes-2

    # # data_path=$prefix/kinect-white-boxes-1 # not good
    # data_path=$prefix/backup-1/kinect-white-boxes-2 # good slammap

    # data_path=$prefix/corridor-4 # good

    # data_path=$prefix/kitchen-1
    # data_path=$prefix/kitchen-2 # slammap quite ok

    # # results not clean enough
    # data_path=$prefix/kitchen-3 # good slammap

    # data_path=$prefix/seminar-3 # ok

    filename=kinect_recorder
    # kinect camera
    camera=" -c 529.21508098293293 -c 525.56393630057437 -c 328.94272028759258 -c 267.48068171871557 "
    # record_nframes=500 # stair case
  ;;

  "calib")
    prefix=$data_path_prefix/data/depth_calib/primesense

    data_path=$prefix/dots-1
    data_path=$prefix/dots-3
    data_path=$prefix/test-1.5-2-3-4-5 # 0, 31, 71, 110, 137
    filename=kinect_recorder
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

"exp-rgbd")
  input_path=$data_path
  # input_path=$prefix/20140612-ceiling-1
  file_prefix=sensor_recorder_000172
  exp_rgbd \
  --cam_file E:/Data/depth_calib/primesense/clams \
  --color_file $input_path/$file_prefix-color.png \
  --depth_file $input_path/$file_prefix-depth.png
  ;;


"gen")
  generate_slammap \
  --cam_file $data_path/camera.txt \
  --rec $data_path/$filename.txt \
  --traj_file "1" \
  --slammap_file $data_path/clams/clams-slammap.bin \
  --pointcloud $data_path/clams/clams-cloud.pcd \
  --resolution 0.01 --max_range 2.0 --skip_poses 10
  ;;

"exp-slmap")
  exp_slammap --slammap_file $data_path/clams/clams-slammap.bin \
  --pointcloud $data_path/clams/clams-cloud.pcd 
  ;;

"calib")
  calibrate --increment 5 --workspace $prefix \
  --calib_params 1 --calib_params 5 --calib_params 4 --calib_params 0.068
  ;;

"vis-mo")
  visualize_model --intrinsics $prefix/clams/distortion_model.bin
  ;;

"undist")
  input_path=$data_path
  # input_path=$prefix/20140612-ceiling-1
  file_prefix=sensor_recorder_000000 # 1.5meter
  axis_length=1.5
  # file_prefix=sensor_recorder_000031 # 2meter
  # axis_length=2
  # file_prefix=sensor_recorder_000071 # 3meter
  # axis_length=3
  file_prefix=sensor_recorder_000110 # 4meter
  axis_length=4
  file_prefix=sensor_recorder_000140 # 4.5meter
  axis_length=4.5
  # file_prefix=sensor_recorder_000172
  undistort --intrinsics E:/Data/depth_calib/primesense/clams/distortion_model.bin \
  --cam_file E:/Data/depth_calib/primesense/clams \
  --color_file $input_path/$file_prefix-color.png \
  --depth_file $input_path/$file_prefix-depth.png \
  --axis_length $axis_length
  ;;

esac


cd "$current_path"
