ARG BASE_IMAGE=ros:humble

FROM ${BASE_IMAGE} AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep \
    && rosdep init || true

# get the source tree and analyse it for its package.xml only
FROM base AS sourcefilter
COPY ./src /tmp/src
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base AS depbuilder
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt-get update \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y && rm -rf /tmp/src

FROM depbuilder AS devcontainer
ARG GIT_REPO_VERSION="unknown"
ARG GIT_REPO_VERSION_DATE="unknown"

# export docker image build information to environment
ENV GIT_REPO_VERSION=${GIT_REPO_VERSION}
ENV GIT_REPO_VERSION_DATE=${GIT_REPO_VERSION_DATE}

FROM depbuilder AS final
ARG GIT_REPO_VERSION="unknown"
ARG GIT_REPO_VERSION_DATE="unknown"

# export docker image build information to environment
ENV GIT_REPO_VERSION=${GIT_REPO_VERSION}
ENV GIT_REPO_VERSION_DATE=${GIT_REPO_VERSION_DATE}

RUN mkdir -p /root/workspace/src
COPY . /root/workspace/src/repository
WORKDIR /root/workspace/
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt-get update \
    && rosdep install --from-paths ./src --ignore-src -r -y \
    && bash -c "source /opt/ros/${ROS_DISTRO}/setup.sh; colcon build"

