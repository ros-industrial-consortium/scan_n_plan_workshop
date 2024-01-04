ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract_ros2:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

ARG ROS_DISTRO

# Install Python dependencies
RUN python3 -m pip install open3d pyquaternion

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/snp
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp \
  source /opt/tesseract_ros2/install/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/snp/dependencies_${ROS_DISTRO}.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
    --skip-keys libvtk \
  && apt install -y \
    libxmlrpcpp-dev

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp \
  source /opt/tesseract_ros2/install/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args \
  && rm -rf build log
