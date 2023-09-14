#!/bin/bash
find_ws=$(pwd)/../../..

lidar_types_rs=("velo_16" "velo_32" "velo_128" "os_32" "os_64" "os_128")
lidar_types_nrs=("livox_horizon" "livox_mid70" "livox_mid40" "livox_avia")
camera_types=("rgb" "thermal")
suite_types=("ll" "lc")

# lidar type: 1:repetitive scan lidar 2:no-repetitive scan lidar 3: camera
sensor_type_code1=0
sensor_type_code2=0
#suite_type: 0: lidar to lidar 1: lidar to camera
suite_type_code=-1

# tp1="/livox/lidar"
# ns1="livox_mid70"
# tp2="/ouster/points"
# ns2="os_32"

# tp1="/livox/lidar"
# ns1="livox_horizon"
# tp2="/rgb_cam/image_raw"
# ns2="rgb"
# cam_param_path="/home/jun/catkin_ws_lyy/src/thermalvox_calib/data/camera_info/horizon_thermal.txt"

cam_param_path="null"
is_dark_board="false"
is_compressed_image="false"
max_frame=60

funcInputInfo()
{
    echo ""
    echo "Pls choose namespace from followings:"
    echo "-LiDAR: [${lidar_types_rs[*]} ${lidar_types_nrs[*]}]"
    echo -e "-Camera: [${camera_types[*]}]\n"

    read -p "sensor1 topic (only LiDAR): " tp1
    read -p "namespace1: " ns1
    read -p "sensor2 topic (LiDAR or Camera): " tp2
    read -p "namespace2: " ns2
}

funcStartProcessLL()
{
    l1_tp=$1
    l1_ns=$2
    l2_tp=$3
    l2_ns=$4
    l1_type=$l1_ns
    l2_type=$l2_ns


    if [ $l1_ns = $l2_ns ]
    then
        l1_ns="${l1_ns}_1"
        l2_ns="${l2_ns}_2"
    fi

    # pattern detection
    gnome-terminal --title="${l1_ns}_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib ${l1_type}_pattern.launch cloud_tp:=${l1_tp} ns_:=${l1_ns}"
    gnome-terminal --title="${l2_ns}_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib ${l2_type}_pattern.launch cloud_tp:=${l2_tp} ns_:=${l2_ns}"

    # feature collection
    gnome-terminal --title="l2l_collection" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib pattern_collection.launch l2l_calib:=true ns_s1:=${l1_ns} ns_s2:=${l2_ns} max_frame:=${max_frame};exec bash"

    # calibration
    source $find_ws/devel/setup.bash
    roslaunch lvt2calib extrinsic_calib.launch l2l_calib:=true ns_s1:=${l1_ns} ns_s2:=${l2_ns} is_auto_mode:=true
}

funcStartProcessLC()
{
    l_tp=$1
    l_ns=$2
    c_tp=$3
    c_ns=$4
    cam_info_path=$5

    # pattern detection
    gnome-terminal --title="${l_ns}_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib ${l_ns}_pattern.launch cloud_tp:=${l_tp} ns_:=${l_ns}"
    gnome-terminal --title="${c_ns}_pattern" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib ${c_ns}_cam_pattern.launch image_tp:=${c_tp} ns_:=${c_ns} cam_info_dir:=${cam_info_path} ifCompressed:=${is_compressed_image} isDarkBoard:=${is_dark_board}"
    
    # feature collection
    gnome-terminal --title="l2c_collection" -- bash -c "source $find_ws/devel/setup.bash;roslaunch lvt2calib pattern_collection.launch l2c_calib:=true ns_s1:=${l_ns} ns_s2:=${c_ns} max_frame:=${max_frame};exec bash"

    # calibration
    source $find_ws/devel/setup.bash
    roslaunch lvt2calib extrinsic_calib.launch l2c_calib:=true ns_s1:=${l_ns} ns_s2:=${c_ns} cam_info_path:=${cam_info_path} is_auto_mode:=true
}

ARGS=`getopt -o "dcf:" -l "darkBoard,compressedImg,maxFrame:" -n "start_up.bash" -- "$@"`
eval set -- "${ARGS}"

while true; do
    case "${1}" in
        -d|--darkBoard)
        is_dark_board="true"
        echo "--use dark board"
        shift;
        ;;
        -c|--compressedImg)
        is_compressed_image="true"
        echo "--use compressed image"
        shift;
        ;;
        -f|--maxFrame)
        shift;
        if [[ -n "${1}" ]]; then
            max_frame=$1
            echo "--max collection frame: $max_frame"
            shift;
        fi
        ;;
        --)
        shift;
        break;
        ;;
    esac
done

funcInputInfo

for i in ${lidar_types_rs[*]}
do
    if [ $ns1 = $i ]
    then 
        sensor_type_code1=1
        lidar_brand1=${ns1%_*}
        lidar_scan_line1=${ns1#*_}
    fi
    if [ $ns2 = $i ]
    then 
        sensor_type_code2=1
        lidar_brand2=${ns2%_*}
        lidar_scan_line2=${ns2#*_}
        suite_type_code=0
    fi
done

if [[ $sensor_type_code1 == 0 || $sensor_type_code2 == 0 ]]
then
    for i in ${lidar_types_nrs[*]}
    do
        if [[ $sensor_type_code1 == 0 && $ns1 = $i ]]
        then 
            sensor_type_code1=2
            lidar_brand1=${ns1%_*}
            lidar_scan_line1=${ns1#*_}
        fi
        if [[ $sensor_type_code2 == 0 && $ns2 = $i ]]
        then 
            sensor_type_code2=2
            lidar_brand2=${ns2%_*}
            lidar_scan_line2=${ns2#*_}
            suite_type_code=0
        fi
    done
fi


if [ $sensor_type_code2 == 0 ]
then
    for i in ${camera_types[*]}
    do
        if [ $ns2 = $i ]
        then
            sensor_type_code2=3
            suite_type_code=1
            break
        fi
    done
fi

echo ""
case $sensor_type_code1 in
    1)
    echo "sensor1_type: repetitive scanning lidar"
    ;;
    2)
    echo "sensor1_type: non-repetitive scanning lidar"
    ;;
    *)
    echo "[ERROR] invalid sensor1 namespace"
    exit 0
    ;;
esac
echo "lidar1 brand: $lidar_brand1"
echo "lidar1 scan line: $lidar_scan_line1"

case $sensor_type_code2 in
    1)
    echo "sensor2_type: repetitive scanning lidar"
    echo "lidar2 brand: $lidar_brand2"
    echo "lidar2 scan line: $lidar_scan_line2"
    ;;
    2)
    echo "sensor2_type: non-repetitive scanning lidar"
    echo "lidar2 brand: $lidar_brand2"
    echo "lidar2 scan line: $lidar_scan_line2"
    ;;
    3)
    echo "sensor2_type: ${ns2} camera"
    ;;
    *)
    echo "[ERROR] invalid sensor2 namespace"
    exit 0
    ;;
esac

# echo "suite_type:" ${suite_types[$suite_type_code]}

if [[ $sensor_type_code1 > 0 && $sensor_type_code2 > 0 ]]
then
    case $suite_type_code in
        0)
        echo -e "LiDAR to LiDAR Calibration\n"
        funcStartProcessLL $tp1 $ns1 $tp2 $ns2
        ;;
        1)
        echo -e "LiDAR to Camera Calibration"
        read -p "camera intrinsic param filepath: " cam_param_path
        echo ""
        funcStartProcessLC $tp1 $ns1 $tp2 $ns2 $cam_param_path
        ;;
    esac
fi