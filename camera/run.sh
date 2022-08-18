#!/bin/bash

echo "Sart time=$(date +"%T")"

IMG_NAME="hihi313/ros-ceres"
IMG_TAG="latest"
CTNR_NAME="camera_ctnr"
WORKDIR="/catkin_ws/src/"

VOLUME=""
while getopts "v:r:e" opt
do
  case $opt in
    v)
        VOLUME="-v $OPTARG"
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

            # --volume="${PWD}:${WORKDIR}" \
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
            --mount type=volume,src="vscode-extensions",dst="/root/.vscode-server/extensions" \
            $VOLUME \
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
