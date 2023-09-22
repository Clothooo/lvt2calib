### ISSUES

------

#### Q1: The feature detection in camera failed (step 3.2).

1. Confirm the calibration board's color matched what was set during the execution of start_up.bash (`darkBoard` or `whiteBoard`)
2. Ensure enough board-background contrast.
3. Watch for board reflections in reflective environments (especially for the thermal camera)

#### Q2: The board detetcion in LiDAR failed (step 3.2)

1. **The LiDAR intensity differs from code presets**. Check LiDAR's row point cloud in Rviz for intensity on the calibration board (assumed as *n*) and on the poster support (assumed as *m*). Typically, *m*<*n*. Run

   ```shell
   rosrun rqt_reconcfigure rqt_reconcfigure
   ```

   find the `(ns_lidar)/laser_pattern` node and adjust '*i_filter_out_max*' value between *m* and *n*, until the detected calibration board is successfully shown in Rviz.

2. The preset clustering parameters are not suitable, which can happen when **the clustering 'min size' threshold is set too low**, leading to the filtering out of the calibration board point cloud. This situation often occurs when calibrating sparse lidars like Velodyne VLP-16. Please adjust the value of '*cluster_size_min*' based on the actual situation. 

   (Note: this value is not a specific point cloud size but a ratio.)

3. (Not recommend to prioritize) The threshold for board detection is too strict. You can adjust the values of '*rmse_ukn2tpl_thre*' and '*rmse_tpl2ukn_thre*'. The larger these values are, the more lenient the detection criteria become. However, please note that these values will affect the accuracy of board detection. It is recommended to adjust them in the range of [0.2, 0.5].

   

#### Q3: The feature collection takes too long (step 3.3)

1. The default number of collection frame in the code is 60 frames, which has been found to be optimal through experiments. You can reduce this value as needed by specifying the '*maxFrame*' argument when running `start_up.bash`

   ```shell
   # set the number of collection frame as 30
   bash start_up.bash --maxFrame 30
   ```

   (Note: Reducing this variable may affect calibration accuracy)

2. The LiDAR's field of view is too large, for example, in the case of a 360-degree scanning LiDAR, resulting in too many outlier point clouds, which adds unnecessary computational burden. You can consider preprocessing the raw point cloud using tools like a passthrough filter.





### NODE STRUCTURE

------



### TOPIC DESCRIPTION

------



### PARAMETER DESCRIPTION

------

