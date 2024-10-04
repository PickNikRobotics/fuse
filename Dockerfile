# Docker setup that's used for CI.

FROM osrf/ros:rolling-desktop-full

# Install external packages.
# hadolint ignore=DL3008
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
      clang-tidy \
      # use cyclonedds instead of fastdds
      ros-rolling-rmw-cyclonedds-cpp

# Create the colcon ws. For now, copy the source files into the workspace
# so that we don't have to deal with cloning this repo, which is private.
WORKDIR /colcon_ws/src/fuse
COPY . .
WORKDIR /colcon_ws
# hadolint ignore=SC1091
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    --mount=type=cache,target=/$HOME/.cache,sharing=locked \
    apt-get update && apt-get upgrade -y && \
    . /opt/ros/rolling/setup.sh && \
    rosdep install --from-paths src -y --ignore-src && \
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
