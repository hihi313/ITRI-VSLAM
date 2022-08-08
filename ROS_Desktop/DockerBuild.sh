#!/bin/bash

echo "Sart time=$(date +"%T")"
IMG_NAME="my_ros"
IMG_TAG="latest"
CTNR_NAME="ros_ctnr"
CTNR_BASE_DIR="/app"

while getopts "t:b:r:e" opt
do
  case $opt in
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
        # --mount type=volume,src="",dst="" \
        # --mount type=bind,src="",dst="" \
        sudo xhost +local:root
        docker run $RM -it --init \
            --ipc=host \
            --user="$(id -u):$(id -g)" \
            -e "DISPLAY" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="$PWD:/app" \
            --name $CTNR_NAME \
            "$IMG_NAME:$IMG_TAG"
        # Disable tracing
        set +x        
        ;;
    e)
        docker exec -it --user=root $CTNR_NAME bash
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