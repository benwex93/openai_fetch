FROM osrf/ros:melodic-desktop-full

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
        ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
        ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compat32,utility

################################################
#NANO TEXT EDITOR and other utilities
################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    net-tools \
    iputils-ping \
  && rm -rf /var/lib/apt/lists/*


COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]