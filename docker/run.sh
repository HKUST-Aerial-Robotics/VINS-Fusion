#!/bin/bash
trap : SIGTERM SIGINT

function echoUsage()
{
    echo -e "Usage: ./run.sh [FLAG] LAUNCH_FILE \n\
            \t -l with loop fusion \n\
            \t -g with global fusion \n\
            \t -k with kitti \n\
            \t -h help" >&2
}

function absPath() 
{
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

function relativePath()
{
    # both $1 and $2 are absolute paths beginning with /
    # returns relative path to $2/$target from $1/$source
    source=$1
    target=$2

    common_part=$source # for now
    result="" # for now

    while [[ "${target#$common_part}" == "${target}" ]]; do
        # no match, means that candidate common part is not correct
        # go up one level (reduce common part)
        common_part="$(dirname $common_part)"
        # and record that we went back, with correct / handling
        if [[ -z $result ]]; then
            result=".."
        else
            result="../$result"
        fi
    done

    if [[ $common_part == "/" ]]; then
        # special case for root (no common path)
        result="$result/"
    fi

    # since we now have identified the common part,
    # compute the non-common part
    forward_part="${target#$common_part}"

    # and now stick all parts together
    if [[ -n $result ]] && [[ -n $forward_part ]]; then
        result="$result$forward_part"
    elif [[ -n $forward_part ]]; then
        # extra slash removal
        result="${forward_part:1}"
    fi

    echo $result
}

if [ "$#" -lt 1 ]; then
  echoUsage
  exit 1
fi

LOOP_FUSION=0
GLOBAL_FUSION=0
KITTI=0

while getopts "hglk" opt; do
    case "$opt" in
        h)
            echoUsage
            exit 0
            ;;
        g)  GLOBAL_FUSION=1
            ;;
        l)  LOOP_FUSION=1
            ;;
        k)  KITTI=1
            ;;
        *)
            echoUsage
            exit 1
            ;;
    esac
done

if [ $KITTI -eq 0 ]; then
    CONFIG_IN_DOCKER="/root/catkin_ws/src/VINS-Fusion/$(relativePath $(absPath ..) $(absPath ${*: -1}))"
else
    CONFIG_IN_DOCKER="/root/catkin_ws/src/VINS-Fusion/$(relativePath $(absPath ..) $(absPath ${*: -2:1}))"
    KITTI_DATASET="$(absPath ${*: -1})"
fi

roscore &
ROSCORE_PID=$!
sleep 1

rviz -d ../config/vins_rviz_config.rviz &
RVIZ_PID=$!

VINS_FUSION_DIR=$(absPath "..")

if [ $KITTI -eq 0 ]; then
    if [ $LOOP_FUSION -eq 0 ]; then
        docker run \
        -it \
        --rm \
        --net=host \
        -v ${VINS_FUSION_DIR}:/root/catkin_ws/src/VINS-Fusion/ \
        ros:vins-fusion \
        /bin/bash -c \
        "cd /root/catkin_ws/; \
        catkin config \
                --env-cache \
                --extend /opt/ros/$ROS_DISTRO \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release; \
            catkin build; \
            source devel/setup.bash; \
            rosrun vins vins_node ${CONFIG_IN_DOCKER}"
    else
        docker run \
        -it \
        --rm \
        --net=host \
        -v ${VINS_FUSION_DIR}:/root/catkin_ws/src/VINS-Fusion/ \
        ros:vins-fusion \
        /bin/bash -c \
        "cd /root/catkin_ws/; \
        catkin config \
                --env-cache \
                --extend /opt/ros/$ROS_DISTRO \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release; \
            catkin build; \
            source devel/setup.bash; \
            rosrun loop_fusion loop_fusion_node ${CONFIG_IN_DOCKER} & \
            rosrun vins vins_node ${CONFIG_IN_DOCKER}"
    fi
else
    if [ $LOOP_FUSION -eq 1 ]; then
        docker run \
        -it \
        --rm \
        --net=host \
        -v ${VINS_FUSION_DIR}:/root/catkin_ws/src/VINS-Fusion/ \
        -v ${KITTI_DATASET}:/root/kitti_dataset/ \
        ros:vins-fusion \
        /bin/bash -c \
        "cd /root/catkin_ws/; \
        catkin config \
                --env-cache \
                --extend /opt/ros/$ROS_DISTRO \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release; \
            catkin build; \
            source devel/setup.bash; \
            rosrun loop_fusion loop_fusion_node ${CONFIG_IN_DOCKER} & \
            rosrun vins kitti_odom_test ${CONFIG_IN_DOCKER} /root/kitti_dataset/"
    elif [ $GLOBAL_FUSION -eq 1 ]; then
        docker run \
        -it \
        --rm \
        --net=host \
        -v ${VINS_FUSION_DIR}:/root/catkin_ws/src/VINS-Fusion/ \
        -v ${KITTI_DATASET}:/root/kitti_dataset/ \
        ros:vins-fusion \
        /bin/bash -c \
        "cd /root/catkin_ws/; \
        catkin config \
                --env-cache \
                --extend /opt/ros/$ROS_DISTRO \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release; \
            catkin build; \
            source devel/setup.bash; \
            rosrun global_fusion global_fusion_node & \
            rosrun vins kitti_gps_test ${CONFIG_IN_DOCKER} /root/kitti_dataset/"
    else
        docker run \
        -it \
        --rm \
        --net=host \
        -v ${VINS_FUSION_DIR}:/root/catkin_ws/src/VINS-Fusion/ \
        -v ${KITTI_DATASET}:/root/kitti_dataset/ \
        ros:vins-fusion \
        /bin/bash -c \
        "cd /root/catkin_ws/; \
        catkin config \
                --env-cache \
                --extend /opt/ros/$ROS_DISTRO \
            --cmake-args \
                -DCMAKE_BUILD_TYPE=Release; \
            catkin build; \
            source devel/setup.bash; \
            rosrun vins kitti_odom_test ${CONFIG_IN_DOCKER} /root/kitti_dataset/"
    fi
fi

wait $ROSCORE_PID
wait $RVIZ_PID

if [[ $? -gt 128 ]]
then
    kill $ROSCORE_PID
    kill $RVIZ_PID
fi
