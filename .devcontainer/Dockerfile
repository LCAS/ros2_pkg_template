ARG BASE_IMAGE=lcas.lincoln.ac.uk/devcontainer/ros2-teaching:4

FROM ${BASE_IMAGE} as base

USER root

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get upgrade -y 

# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
COPY ./src /tmp/src
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as depbuilder
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update --rosdistro ${ROS_DISTRO} && apt-get update
RUN cd /tmp/src && rosdep install --from-paths . --ignore-src -r -y && cd && rm -rf /tmp/src

FROM depbuilder as final
# add user ros with sudo rights if it doesn't exist

# add sudo without password
ENV DEBIAN_FRONTEND=noninteractive

USER ros