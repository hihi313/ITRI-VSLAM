#!/bin/bash

echo "Sart time=$(date +"%T")"

IMG_NAME="hihi313/slam"
IMG_TAG="latest"
CTNR_NAME="slam_ctnr"
WORKDIR="/app" # Should be the same as Dockerfile

while getopts "i:t:b:r:e" opt; do
    case $opt in
    i)
        IMG_NAME="$OPTARG"
        ;;
    t)
        IMG_TAG="$OPTARG"
        ;;
    b)
        if [ "$OPTARG" == "n" ]; then
            CACHE="--no-cache"
        else
            CACHE=""
        fi
        START="$(TZ=UTC0 printf '%(%s)T\n' '-1')" # `-1`  is the current time

        docker rmi $IMG_NAME
        docker build $CACHE -t $IMG_NAME:$IMG_TAG

        # Pring elapsed time
        ELAPSED=$(($(TZ=UTC0 printf '%(%s)T\n' '-1') - START))
        TZ=UTC0 printf 'Build duration=%(%H:%M:%S)T\n' "$ELAPSED"
        ;;
    r)
        RM=""
        GPU=""
        DISPLAY_ENV="DISPLAY=$DISPLAY"
        DISPLAY_VOLUME="--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw"
        if [[ $OPTARG == *"m"* ]]; then
            RM="--rm"
        fi
        if [[ $OPTARG == *"g"* ]]; then
            GPU="--gpus all"
        fi
        if [[ $OPTARG == *"d"* ]]; then
            DISPLAY_ENV="DISPLAY=host.docker.internal:0"
            DISPLAY_VOLUME=""
        fi
        # Enable tracing
        set -x
        # --mount type=volume,src="",dst="" \
        # --mount type=bind,src="",dst="" \
        # --user="$(id -u):$(id -g)" \

        sudo xhost +local:root &&
            docker run -it $RM $GPU $DISPLAY_VOLUME \
                -e QT_X11_NO_MITSHM=1 \
                -p 8088:8088 \
                -v /dev:/dev:ro \
                --privileged \
                --init \
                --ipc=host \
                --env="$DISPLAY_ENV" \
                --mount type=volume,src="vscode-extensions",dst="/root/.vscode-server/extensions" \
                --volume="$PWD:$WORKDIR" \
                --workdir $WORKDIR \
                --name $CTNR_NAME \
                "$IMG_NAME:$IMG_TAG"

        # Disable tracing
        set +x
        ;;
    e)
        # Enable tracing
        set -x
        docker exec -it $CTNR_NAME bash
        # Disable tracing
        set +x
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
