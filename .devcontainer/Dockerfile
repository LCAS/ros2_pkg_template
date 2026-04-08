ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-latest
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE} AS devcontainer

RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
    ros-${ROS_DISTRO}-desktop \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

COPY ./ /workspace/
WORKDIR /workspace

ENV ROS2_WS=/workspace
RUN rosdep update --rosdistro ${ROS_DISTRO} &&\
    apt-get update && \
    rosdep install --from-paths ./src --ignore-src -r -y

# Do an initial build to speed up development
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --continue-on-error"

# ----------------------------------------------------------------------------------------------------
FROM devcontainer AS final

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

CMD ["/bin/bash", "-lc", "source /workspace/install/setup.bash && exec ros2 launch example example.launch.py"]
