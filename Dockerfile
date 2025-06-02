# Docker setup that's used for CI.

FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb

# Install external packages.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      clang-tidy \
      python3-vcstool \
      # use cyclonedds instead of fastdds
      ros-humble-rmw-cyclonedds-cpp

# Create the colcon ws. For now, copy the source files into the workspace
# so that we don't have to deal with cloning this repo, which is private.
WORKDIR /colcon_ws/src/fuse
COPY . .
WORKDIR /colcon_ws
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    . /opt/ros/humble/setup.sh && \
    rosdep install --from-paths src -y --ignore-src && \
    # tf2_2d testing build fails due to upstream tf2 changes, it seems
    colcon build --mixin compile-commands coverage-gcc coverage-pytest

# Set up final environment and entrypoint.
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI /root/.ros/cyclonedds.xml
COPY dds/cyclonedds_local.xml $CYCLONEDDS_URI
COPY .clang-tidy /colcon_ws
COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENV SHELL /bin/bash
CMD ["/bin/bash"]
