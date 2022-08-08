#!/bin/bash

echo "Sart time=$(date +"%T")"
IMG_NAME="my_ros"
IMG_TAG="latest"
CTNR_NAME="ros_ctnr"
CTNR_BASE_DIR="/app"

while getopts "t:br:e" opt
do
  case $opt in
    t)
        IMG_TAG="$OPTARG"
        ;;
    b) 
        START="$(TZ=UTC0 printf '%(%s)T\n' '-1')" # `-1`  is the current time
        
        docker rmi $IMG_NAME
        docker build --no-cache -t $IMG_NAME .
        
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
        sudo docker run --privileged \
                $RM \
                -it \
                --name $CTNR_NAME \
                # --mount type=volume,src="",dst="" \
                # --mount type=bind,src="",dst="" \
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