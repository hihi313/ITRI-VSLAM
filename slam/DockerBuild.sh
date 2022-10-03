#!/bin/bash

echo "Sart time=$(date +"%T")"

IMG_NAME="lmwafer/orb-slam-3-ready"
IMG_TAG="1.1-ubuntu18.04"
CTNR_NAME="orb_slam3_ctnr"
WORKDIR="/dpds" # Should be the same as Dockerfile

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
        docker build $CACHE -t $IMG_NAME .

        # Pring elapsed time
        ELAPSED=$(($(TZ=UTC0 printf '%(%s)T\n' '-1') - START))
        TZ=UTC0 printf 'Build duration=%(%H:%M:%S)T\n' "$ELAPSED"
        ;;
    r)
        if [ "$OPTARG" == "m" ]; then
            RM="--rm"
        else
            RM=""
        fi
        # Enable tracing
        set -x
        # --mount type=volume,src="",dst="" \
        # --mount type=bind,src="",dst="" \
        # --user="$(id -u):$(id -g)" \
        # --mount type=volume,src="apt-list",dst="/var/lib/apt/lists/" \
        sudo xhost +local:root
        docker run --privileged \
            $RM \
            -it \
            --ipc=host \
            -p 8087:8087 \
            -e DISPLAY="host.docker.internal:0" \
            -e QT_X11_NO_MITSHM=1 \
            --gpus all \
            -v /dev:/dev:ro \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --mount type=volume,src="vscode-extensions",dst="/root/.vscode-server/extensions" \
            --volume="$PWD:$WORKDIR/ORB_SLAM3/src" \
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
