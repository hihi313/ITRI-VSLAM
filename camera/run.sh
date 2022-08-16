#!/bin/bash

echo "Sart time=$(date +"%T")"

IMG_NAME="hihi313/ros-kalibr-ceres"
IMG_TAG="latest"
CTNR_NAME="camera_ctnr"
WORKDIR="/catkin_ws/src"

while getopts "i:t:b:r:e" opt
do
  case $opt in
    r)
        if [ "$OPTARG" == "m" ]
        then
            RM="--rm"
        else
            RM=""
        fi

        # Enable tracing
        set -x

        sudo xhost +local:root
        docker run \
            $RM \
            -it \
            -e DISPLAY=$DISPLAY \
            -e "QT_X11_NO_MITSHM=1" \
            -v /dev/bus/usb:/dev/bus/usb \
            --init \
            --privileged \
            --device-cgroup-rule='c 189:* rmw' \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="${PWD}:${WORKDIR}" \
            --name $CTNR_NAME \
            "${IMG_NAME}:${IMG_TAG}" \
            bash

        # Disable tracing
        set +x        
        ;;
    e)
        docker exec -it $CTNR_NAME bash
        ;;
    \?) 
        echo "Invalid option -$OPTARG" >&2
        exit 1
        ;;
    :)
        echo "Option -$OPTARG requires an argument." >&2
        exit 1
        ;;
    *)
        echo "*"
        ;;
  esac
done
