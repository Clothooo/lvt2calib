# LVT2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/lvt2_sensor_trans_fig.png" style="zoom:80%;" />

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/qualification_theraml_livox.png" style="zoom:80%;" />

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/qualification_ouster_livox.png" style="zoom: 80%;" />

## Introduction

This solution provides a automatic an unified method for extrinsic calibration between repetitive scanning and non-repetitive scanning 3D LiDAR, sparse and dense 3D LiDAR, visual  and thermal camera.

A four-circular-holes board is adopted for all sensors as the calibration board. The four circle canters can be detected by all sensors, thus are ideal common features. To unify the calibration process, we propose a automatic target detection method based on template matching. In addition, we provide two types of output, minimizing 2D re-projection error (Min2D) and minimizing 3D matching error (Min3D), for different users of LiDAR-Camera suite.

## How to use

### Step1: Environment Configuration

#### 1.1 Install environment and driver

Install the ROS  environment and install the SDK and driver of the LiDAR you use. You can skip this step if they are already installed.

#### 1.2 Denpendency

Tested with Ubuntu 20.04 64-bit.

- ROS noetic
- PCL 1.10
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Ceres-solver](http://ceres-solver.org/) (1.14.0)
- OpenCV 4.2.0

### Step2: Preparation

#### 2.1 Download and installation

Download this repo and compile it.

```
git clone https://github.com/Clothooo/lvt2calib.git
...
cd path_to_your_lvt2calib_ws
catkin_make
source devel/setup.bash
```

#### 2.2 Program node brief

This project includes the following nodes:

- `cam_pattern`: automatic detection of the calibration target and extraction of four-circular features in images (for **visual** and **thermal camera**)
- `livox_pattern`: automatic detection of the calibration target and extraction of four-circular features in Livox LiDAR point clouds (generalized to other **non-repetitive scanning 3D LiDAR**)
- `velodyne_pattern`: automatic detection of the calibration target in Velodyne LiDAR (**repetitive scanning 3D LiDAR**) point clouds
- `velodyne_parttern_circle`: extraction of four-circular features in Velodyne LiDAR point clouds.
- `ouster_pattern` & `ouster_pattern_circle`: similar to `velodyne_pattern` and `velodyne_pattern_circle`, designed for Ouster LiDAR (**repetitive scanning 3D LiDAR**)
- `pattern_collection_lc` & `pattern_collection_ll`:  accumulation and collection of features extracted from LiDAR point clouds and camera images (lc), or from two different LiDAR's point clouds(ll)
- `extrinsic_calib_l2c` & `extrinsic_calib_l2l`: calculation of the extrinsic parameters of LiDAR-Camera suite (l2c) or LiDAR-LiDAR suite (l2l)

Some auxiliary node:

- `point_cloud_accumulation`: point cloud integration
- `point_image_project_rebuild:`to re-projection point cloud on images and rebuild the colored point cloud
- `point_cloud_fusion` : to fuse point clouds from multiple LiDARs

#### 2.3 Preparing the calibration board

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/fed39af76a181cf84207adeda7cf71b.jpg" style="zoom: 50%;" />

We use a four-circle plate which has the same size as the plate in our previous work[[1]](https://ieeexplore.ieee.org/document/8961462) (also inspired by Guindel's work[[2]](https://ieeexplore.ieee.org.remotexs.ntu.edu.sg/document/8317829)). It is made by acrylic. Then, the back of the board is attached with the same size heating silicone pad to facilitate thermal imaging.

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/ThermalVoxCalib%20-%20fig_board.png" alt="ThermalVoxCalib - fig_board" style="zoom:80%;" />

In order to ensure a smooth and successful calibration, the following matters need to be noted:

1. It will be better to select a relatively empty environment without obvious reflection on the ground for the calibration scene;
2. There should not be obvious reflection on the ground (to avoid interference of reflection in the view of the thermal camera);
3. The heating temperature of the calibration plate should not be too high to prevent deformation, but also to have a certain gap with the ambient temperature;
4. The calibration board should be placed on a supporter, which should be as stable as possible during the calibration process;
5. The distance from the thermal camera to the calibration board should be <= 6m (considering the image clarity of the thermal camera).

### Step3: Calibration

#### 3.0 Camera Parameter Setting

(If cameras used)

The camera parameter should be saved as `xxx.txt` in folder `(lvt2calib path)/data/camera_info`. The file should be in the format as:

![2021-10-20 18-57-13屏幕截图](https://gitee.com/Clothooo/mypicgo/raw/master/mypicgo/2021-10-20%2018-57-13%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 3.1 Quick Start

run

```shell
roscd lvt2calib/launch/
bash start_up.bash
```

When there is a camera in the suite that needs to be calibrated:

- if the calibration board use is **dark** (default as white), please run

  ```shell
  bash start_up.bash --darkBoard
  ```

- if the **compressed** image topic is used, please run

  ```shell
  bash start_up.bash --compressedImg
  ```

Of course, these two parameters can be used at the same time (no order is required), like:

```shell
bash start_up.bash --compressedImg --darkBoard
```

The terminal feedback:

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/bash_feedback.png)

According to the prompts, enter topic and namespace of the two sensors. <u>Noted: Each sensor corresponds to a specific namespace, please refer to the table in *Appendix* for details.</u> Take the Livox Horizon and RGB camera as an example:

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/bash_input_enter.png)

The type of sensor used is then displayed in the terminal. If it is judged to be a LiDAR-Camera suite, it will continue to prompt for the full path of the camera parameter file (setted in *Step 3.0*).

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/bash_judge_cam_file.png)

Three new terminal windows will pop up, corresponding to *Step 3.2, 3.3 and 3.4*, and The current terminal will enter the process of *Step 3.5*. Please refer to these sections for details.

**Noted**: If <u>two sensors of the same type</u> are used, that is, the input namespace are the same, for example, two Livox Horizon LiDARs (livox_horizon), the program will automatically determine the the namespaces are the same, and <u>add suffixes</u> `_1`and `_2` respectively. This may affect users viewing feature extraction results in *Rviz*, requiring manual changed to the observed topic.

------

(If using quick start, you can skip the following commands.)

#### 3.2 Feature Extraction of Camera

- For the thermal camera, run

  ```shell
  roslaunch lvt2calib thermal_cam_pattern.laucn cam_info_dir:=(path of the camera parameter file) image_tp:=(topic of image) (ns_:=xxx)
  ```

- For the visual camera, run:

  ```shell
  roslaunch lvt2calib rgb_cam_pattern.laucn cam_info_dir:=(path of the camera parameter file) image_tp:=(topic of image) (ns_:=xxx)
  ```

  If the calibration board use is **dark** (default as white), add `isDarkBoard:=true`. If the **compressed** image topic is used, add `ifCompressed:=true`. `ns_` is the node namespace, defined by users.

The program will display the raw image, the undistorted image, the grayed image (visual camera scene) and the result image of circle-center-detection:

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/lvt2calib_circle_camera.png)

**Tips**: If it dose not detect the feature point, please consider whether the calibration board is not heated enough (thermal camera scene), the calibration board is placed at an improper angle or other reasons.

#### 3.3 Feature Extraction of LiDAR

Run 

```shell
roslaunch lvt2calib (liadr_type)_pattern.launch cloud_in_tp:=(topic of point cloud) (ns_:=xxx)
```

`lidar_type` is the type of LiDAR you use. Please refer to the table in *Appendix*.

The program will output the target board founded and four circle centers extracted (after the user starts the feature collection), if the automatic calibration board detection is successful. Results can be viewed in *Rviz*. The description of each topic can be referred in **topics_breif.txt** (TBC...). This picture takes a suite of a Livox Mid70 LiDAR and an Ouster OS1-32 as an example.

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/lvt2calib_circle_laser.png)

You can use passthrough filter to reduce the point cloud size before this step (by setting `use_passthrough_preprocess:=true`), which will speed up this process.

**Tips**: It is recommended to check the topic `/(ns_)/laser_pattern/calib_board_cloud` to make sure the target board can be successfully detected as shown in the following picture, and then proceed to the subsequent operation.

![](https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/lvt2calib_board_laser.png)

#### 3.4 Feature Data Collection

1. Run

   For LiDAR-LiDAR suite:

   ```shell
   roslaunch lvt2calib pattern_collectoin.luanch l2l_calib:=true ns_s1:=aaa ns_s2:=bbb
   ```

   For LiDAR-Camera suite:

   ```shell
   roslaunch lvt2calib pattern_collectoin.luanch l2c_calib:=true ns_s1:=aaa ns_s2:=bbb
   ```

   Arguments reference:

   | sensor suite |             `ns1`              |             `ns2`              |
   | :----------: | :----------------------------: | :----------------------------: |
   | LiDAR-LiDAR  | LiDAR1 node namespace (as 3.3) | LiDAR2 node namespace (as 3.3) |
   | LiDAR-Camera | LiDAR node namespace (as 3.3)  | Camera node namespace (as 3.2) |

2. At the beginning, the program will ask you if it's ready to collect in the terminal. Following 3.2 and 3.3, if everything is ok, press 'y/Y' and 'ENTER' to start. This process will accumulate feature points extracted from sensors. 

3. When the cumulative number of frames reaches the set value `max_acc_frame`, the process will pause and ask whether to proceed to the feature acquisition for the next position and angle of the calibration board![2021-10-20 19-56-37屏幕截图](https://gitee.com/Clothooo/mypicgo/raw/master/mypicgo/2021-10-20%2019-56-37%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

   If yes, please change the position of the calibration board. Then type 'y/Y' and 'Enter' in the terminal for the next round of feature point extraction and collection. 

   If no, type 'n/N' and 'Enter' to end all the above programs. (Note: Please do not directly use *ctrl+c*).

4. If collection ends and quit this process, a file `feature_info_(ns_s1)_to_(ns_s2).csv` and its copy (with a time suffix) will be saved in `(lvt2calib path)/data/pattern_collection_result/`.

#### 3.5 Extrinsic Parameter Calculation

Run

For LiDAR-LiDAR suite:

```shell
roslauch lvt2calib extrinsic_calib.launch l2l_calib:=true ns_s1:=aaa ns_s2:=bbb
```

For LiDAR-Camera suite:

```shell
roslauch lvt2calib extrinsic_calib.launch l2c_calib:=true ns_s1:=aaa ns_s2:=bbb cam_info_path:=(path of the camera parameter file)
```

**Noted**: the `ns_s1` and `ns_s2` must be the same as these in *Step 3.4*.

This program will output the extrinsic parameters and error evaluation, and these will be saved in `(lvt2calib path)/data/calibration_result/` as following files:

- For LiDAR-LiDAR suite:
  - `(ns_s1)_to_(ns_s2)_exParam.csv`: the extrinsic parameters from Sensor1 to Sensor2;
  - `L2L_CalibLog.csv`: the log file of each extrinsic parameter calculation result and errors


- For LiDAR-Camera suite:

  - `(ns_s1)_to_(ns_s2)_exParam_min3d.csv`: the extrinsic parameters from Sensor1 (LiDAR) to Sensor2 (Camera) calculated by minimizing 3d matching error;
  - `(ns_s1)_to_(ns_s2)_exParam_min2d.csv`: the extrinsic parameters from Sensor1 (LiDAR) to Sensor2 (Camera) calculated by minimizing 2d re-projection error;

  - `L2C_CalibLog.csv`: the log file of each extrinsic parameter calculation result and errors;



## Thanks

[1] J. Zhang, R. Zhang, Y. Yue, C. Yang, M. Wen, and D. Wang, “Slat-calib: Extrinsic calibration between a sparse 3d lidar and a limited-fov low-resolution thermal camera,” in *2019 IEEE International Conference on Robotics and Biomimetics (ROBIO)*, pp. 648–653, 2019.

[2] C. Guindel, J. Beltrán, D. Martín, and F. García, “Automatic extrinsic calibration for lidar-stereo vehicle sensor setups,” in *2017 IEEE 20th International Conference on Intelligent Transportation Systems (ITSC)*, pp. 1–6, Oct 2017.



## Appendix

#### I. Table of arguments corresponding to sensors

| No.  |      Sensor      | Type | Namespace (`ns`) | Feature extraction .launch file |
| :--: | :--------------: | :--: | :--------------: | :-----------------------------: |
|  1   |  Livox Horizon   | NRL  |  livox_horizon   |  livox_horizon_pattern.launch   |
|  2   |   Livox Mid 70   | NRL  |   livox_mid70    |   livox_mid70_pattern.launch    |
|  3   |   Livox Mid 40   | NRL  |   livox_mid40    |   livox_mid40_pattern.launch    |
|  4   |    Livox Avia    | NRL  |    livox_avia    |    livox_avia_pattern.launch    |
|  5   | Velodyne VLP-16  | RL_S |     velo_16      |  livox_velo_16_pattern.launch   |
|  6   | Velodyne VLP-32  | RL_S |     velo_32      |  livox_velo_32_pattern.launch   |
|  7   | Velodyne VLP-64  | RL_D |     velo_64      |  livox_velo_64_pattern.launch   |
|  8   | Velodyne VLP-128 | RL_D |     velo_128     |  livox_velo_128_pattern.launch  |
|  9   |  Ouster OS1-32   | RL_D |      os_32       |      os_32_pattern.launch       |
|  10  |  Ouster OS1-64   | RL_D |      os_64       |      os_64_pattern.launch       |
|  11  |  Ouster OS1-128  | RL_D |      os_128      |      os_128_pattern.launch      |
|  12  |    RGB Camera    |  VC  |       rgb        |     rgb_cam_pattern.launch      |
|  13  |  Thermal Camera  |  TC  |     thermal      |   thermal_cam_pattern.launch    |

NRL: Non-repetitive Scanning LiDAR

RL_S: Sparse Repetitive Scanning LiDAR

RL_D: Dense Repetitive Scanning LiDAR

VC: Visual Camera

TC: Thermal Camera

#### II. Parameter Description for Nodes

TBC...

#### III. Paper

[L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera (IEEE Xplore)](https://ieeexplore.ieee.org/document/10186657)

[L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera (ResearchGate)](https://www.researchgate.net/publication/371377845_L2V2T2Calib_Automatic_and_Unified_Extrinsic_Calibration_Toolbox_for_Different_3D_LiDAR_Visual_Camera_and_Thermal_Camera)

## Citation
If you find this work useful for your research, please consider citing:
```
@INPROCEEDINGS{ZhangLiu2023IV,
  author={Zhang, Jun and Liu, Yiyao and Wen, Mingxing and Yue, Yufeng and Zhang, Haoyuan and Wang, Danwei},
  booktitle={2023 IEEE Intelligent Vehicles Symposium (IV)}, 
  title={L2V2T2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera}, 
  year={2023},
  volume={},
  number={},
  pages={1-7},
  doi={10.1109/IV55152.2023.10186657}}
```
