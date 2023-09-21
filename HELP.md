### ISSUES

------

#### Q1: The feature detection in camera failed (step 3.2).

1. Confirm the calibration board's color matched what was set during the execution of start_up.bash (`darkBoard` or `whiteBoard`)
2. Ensure enough board-background contrast.
3. Watch for board reflections in reflective environments (especially for the thermal camera)

#### Q2: The board detetcion in LiDAR failed (step 3.2)

1. The LiDAR intensity differs from code presets. Check LiDAR's row point cloud in Rviz for intensity on the calibration board (assumed as *n*) and on the poster support (assumed as *m*). Typically, *m*<*n*. Run

   ```shell
   rosrun rqt_reconcfigure rqt_reconcfigure
   ```

   find the `(ns_lidar)/laser_pattern` node and adjust '*i_filter_out_max*' value between *m* and *n*, until the detected calibration board is successfully shown in Rviz.

2. To be added...

#### Q3: The feature collection takes too long (step 3.3)

The default number of collection frame in the code is 60 frames, which has been found to be optimal through experiments. You can reduce this value as needed by specifying the '*maxFrame*' argument when running `start_up.bash`

```shell
# set the number of collection frame as 30
bash start_up.bash --maxFrame 30
```



### NODE STRUCTURE

------



### TOPIC DESCRIPTION

------



### PARAMETER DESCRIPTION

------

