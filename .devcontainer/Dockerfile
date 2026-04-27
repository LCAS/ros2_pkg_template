ARG BASE_IMAGE=lcas.lincoln.ac.uk/ros:humble-latest
ARG ROS_DISTRO=humble

FROM ${BASE_IMAGE} AS devcontainer

ENV main_user=${USER:-root}

USER root
RUN rm -rf /var/lib/apt/lists/* \
    && apt-get update --fix-missing \
    && apt-get -y install --no-install-recommends --fix-missing \
        bash-completion \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
ENV ROS2_WS=/workspace
RUN --mount=type=bind,source=.,target=/workspace rosdep update --rosdistro ${ROS_DISTRO} \
    && apt-get update \
    && rosdep install --from-paths /workspace/src --ignore-src -r -y

USER ${main_user}

# Do an initial build to speed up development
RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --continue-on-error"

# ----------------------------------------------------------------------------------------------------
FROM devcontainer AS final

RUN /bin/bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

CMD ["/bin/bash", "-lc", "source /workspace/install/setup.bash && exec ros2 launch example example.launch.py"]