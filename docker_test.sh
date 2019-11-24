
docker build -t docker_test .
xhost +local:root

nvidia-docker run \
    --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    -it \
    -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all \
    docker_test
export containerId=$(docker ps -l -q)
