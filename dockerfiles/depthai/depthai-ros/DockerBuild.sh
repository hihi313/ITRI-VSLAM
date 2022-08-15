#!/bin/bash

echo "Sart time=$(date +"%T")"

IMG_NAME="ros-kalibr-ceres"
IMG_TAG="depthai-ros"
CTNR_NAME="${IMG_NAME}_ctnr"
WORKDIR="/catkin_ws" # Should be the same as Dockerfile

while getopts "i:t:b:r:e" opt
do
  case $opt in
    i)
        IMG_NAME="$OPTARG"
        ;;
    t)
        IMG_TAG="$OPTARG"
        ;;
    b) 
        if [ "$OPTARG" == "n" ]
        then
            CACHE="--no-cache"
        else
            CACHE=""
        fi
        
        START="$(TZ=UTC0 printf '%(%s)T\n' '-1')" # `-1`  is the current time
        
        docker rmi $IMG_NAME
        docker build $CACHE -t $IMG_NAME . 
        
        # Pring elapsed time
        ELAPSED=$(( $(TZ=UTC0 printf '%(%s)T\n' '-1') - START ))
        TZ=UTC0 printf 'Build duration=%(%H:%M:%S)T\n' "$ELAPSED"
        ;;
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
