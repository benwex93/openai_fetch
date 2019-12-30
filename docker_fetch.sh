docker login --username=benwex93 --password=throwaway613
docker build -t benwex93/openai-ros-melodic-opengl:latest .

docker push benwex93/openai-ros-melodic-opengl:latest
xhost +local:root
nvidia-docker run \
    --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --privileged \
    -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e GAZEBO_GUI=true \
    -e GAZEBO_HEADLESS=false \
    --name='fetch_env' \
    benwex93/openai-ros-melodic-opengl:latest
export containerId=$(docker ps -l -q)
