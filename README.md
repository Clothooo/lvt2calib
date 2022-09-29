# LVT2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera

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

Tested with Ubuntu 16.04 64-bit and Ubuntu 18.04 64-bit.

- ROS (tested with kinetic/melodic)
- PCL 1.8
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [Ceres-solver](http://ceres-solver.org/)
- OpenCV 3.3.1

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

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/fed39af76a181cf84207adeda7cf71b.jpg" style="zoom: 20%;" />

We use a four-circle plate which has the same size as the plate in our previous work. It is made by acrylic. Then, the back of the board is attached with the same size heating silicone pad to facilitate thermal imaging.

<img src="https://raw.githubusercontent.com/Clothooo/mypicgo_win/main/img/ThermalVoxCalib%20-%20fig_board.png" alt="ThermalVoxCalib - fig_board" style="zoom:80%;" />

In order to ensure a smooth and successful calibration, the following matters need to be noted:

1. It will be better to select a relatively empty environment without obvious reflection on the ground for the calibration scene;
2. There should not be obvious reflection on the ground (to avoid interference of reflection in the view of the thermal camera);
3. The heating temperature of the calibration plate should not be too high to prevent deformation, but also to have a certain gap with the ambient temperature;
4. The calibration board should be placed on a supporter, which should be as stable as possible during the calibration process;
5. The distance from the thermal camera to the calibration board should be <= 6m (considering the image clarity of the thermal camera).

### Step3: Calibration

#### 3.1 Camera Parameter Setting

###### (If cameras used)

The camera parameter should be saved as `xxx.txt` in folder `(lvt2calib path)/data/camera_info`. The file should be in the format as:

![2021-10-20 18-57-13屏幕截图](https://gitee.com/Clothooo/mypicgo/raw/master/mypicgo/2021-10-20%2018-57-13%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

#### 3.2 Feature Extraction of Camera

- For the thermal camera, run

```
roslaunch lvt2calib cam_pattern.laucn isThermal:=true camParamFile:=xxx.txt
```

- For the visual camera, run

```
roslaunch lvt2calib cam_pattern.luanch isRGB:=true camParamFile:=(name of the camera parameter file.txt)
```

In visual camera scene, you can tune the  `threshold_value` using the dynamic configuration to adjust the result of image binarization.

The program will display the raw image, the undistorted image, the binarized image (visual camera scene) and the result image of circle-center-detection.

Tips: If it dose not detect the feature point, please consider whether the calibration board is not heated enough (thermal camera scene), the calibration board is placed at an improper angle, the binarization threshold is not proper (visual camera scene), or other reasons.

#### 3.3 Feature Extraction of LiDAR

##### 3.3.1 Non-repetitive Scanning LiDAR

If the scan lines of the LiDAR are dense enough, just run

```
roslaunch lvt2calib livox_pattern.launch cloud_in_tp:=aaa (ns:=bbb)
```

Else, we need to do the point cloud integration. Run

```
roslaunch lvt2calib livox_pattern.launch cloud_in_tp:=aaa time_integrate:=true acc_frame:=xx (ns:=bbb)
```

You can give the accumulation frame here, and you can also adjust it in the dynamic configuration.

The program will output the target board founded and four circle centers extracted, if the automatic calibration board detection is successful. Results can be viewed in *Rviz*. The description of each topic can be referred in **topics_breif.txt** (TBC...).

Tips: It is recommended to check the topic `/(ns)/livox_pattern/calib_board_cloud` to make sure the target board can be successfully detected, and then proceed to the subsequent operation.

##### 3.3.2 Repetitive Scanning LiDAR

Run

```
roslaunch lvt2calib (lidar_type)_pattern.launch cloud_in_tp:=aaa lines_count:=xx (ns:bbb)
```

`lidar_type` refers to the LiDAR you use. We now support these types: `velodyne` and `ouster`. `lines_count` is the number of scan lines of the LiDAR you use. You can use passthrough filter to reduce the point cloud size before this step, which will speed up this process.

Outputs and tips are similar to **3.3.1**.

#### 3.4 Feature Data Collection

1. Run

   ```
   roslaunch lvt2calib pattern_collectoin_(suite_type).luanch ns1:=aaa ns2:=bbb
   ```

   Arguments reference:

   | sensor suite | `suite_type` |          `ns1`           |           `ns2`           |
   | :----------: | :----------: | :----------------------: | :-----------------------: |
   | LiDAR-LiDAR  |     `ll`     | LiDAR1 node ns (as 3.3)  |  LiDAR2 node ns (as 3.3)  |
   | LiDAR-Camera |     `lc`     | default or LiDAR node ns | default or Camera node ns |

2. At the beginning, the program will ask you if it's ready to collect in the terminal. Following 3.2 and 3.3, if everything is ok, press 'Y' and 'ENTER' to start. This process will accumulate feature points extracted from sensors. 

3. When the cumulative number of frames reaches the set value `max_acc_frame`, the process will pause and ask whether to proceed to the feature acquisition for the next position and angle of the calibration board![2021-10-20 19-56-37屏幕截图](https://gitee.com/Clothooo/mypicgo/raw/master/mypicgo/2021-10-20%2019-56-37%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)

   If yes, please change the position of the calibration board. Then type y/Y and Enter in the terminal for the next round of feature point extraction and collection. 

   If no, type n/N and Enter to end all the above programs. (Note: Please do not directly use *ctrl+c*).

4. If collection ends and quit this process, a file `feature_info_(suite_type).csv` and its copy (with a time suffix) will be saved in `(lvt2calib path)/data/pattern_collection/`.

#### 3.5 Extrinsic Parameter Calculation

Run

```
roslauch lvt2calib extrinsic_calib_(suite_type).launch
```

This program will output the extrinsic parameters and error evaluation, and these will be saved in `(lvt2calib path)/data/calibration_result/` as following files:

- `Ex_l2l.csv` and `Ex_l2l_(time).csv`: the extrinsic parameters from LiDAR1 to LiDAR2;
- `Ex_l2c_min3d.csv` and `Ex_l2c_min3d_(time).csv`: the extrinsic parameters from LiDAR to Camera calculated by minimizing 3d matching error;
- `Ex_l2c_min2d.csv` and `Ex_l2c_min2d_(time).csv`: the extrinsic parameters from LiDAR to Camera calculated by minimizing 2d re-projection error;
- `calibration_log.csv`: the log file of each extrinsic parameter calculation result and errors;



## Appendix

#### I. Tested LiDAR

| No.  |      LiDAR      | Type | `acc_frame` |
| :--: | :-------------: | :--: | :---------: |
|  1   |  Livox Horizon  | NRL  |     >=5     |
|  2   |  Livox Mid 70   | NRL  |     >=8     |
|  3   |  Livox Mid 40   | NRL  |     >=8     |
|  4   |   Livox Avia    | NRL  |     >=5     |
|  5   | Velodyne VLP-16 |  RL  |      -      |
|  6   |  Ouster OS1-32  |  RL  |      -      |

#### II. Parameter Description for Nodes

TBC...

#### III. Paper

TBC...
