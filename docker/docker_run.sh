
run_alice_container() {
    xhost +local:docker

    export XSOCK=/tmp/.X11-unix
    export XAUTH=$(mktemp)
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    docker run -it --rm \
        --name alice \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$XAUTH:$XAUTH:rw" \
        --net=host \
        --privileged \
        --env="HOST_USERNAME=$(whoami)" \
        --volume="/home/manga/Alice:/home/rosuser/Alice:rw" \
        --volume="/media:/media" \
        --volume="/dev:/dev:rw" \
        --volume="/dev/dri:/dev/dri:rw" \
        --ulimit rtprio=99 \
        --cap-add=sys_nice \
        --device-cgroup-rule "c 81:* rmw" \
        --device-cgroup-rule "c 189:* rmw" \
        \
        --gpus all \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --env="NVIDIA_VISIBLE_DEVICES=all" \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e __NV_PRIME_RENDER_OFFLOAD=1 \
        -e __VK_LAYER_NV_optimus=NVIDIA_only \
        \
        alice 
}

join_alice_container()
{
  docker exec -it alice bash
}