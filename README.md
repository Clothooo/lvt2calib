# LVT2Calib: Automatic and Unified Extrinsic Calibration Toolbox for Different 3D LiDAR, Visual Camera and Thermal Camera

(image of qualifictaion L2L, L2T)

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

#### 2.3 Preparing the calibration board

(image calibration board size)

