ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract_ros2:${TAG}

# Redeclare TAG argument for use later
ARG TAG

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/snp
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp \
  apt update -y -qq \
  && source /opt/tesseract_ros2/install/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/snp/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
    --skip-keys libvtk python3-open3d

# Install open3d
RUN if [[ "${TAG}" < "jazzy" ]] ; then apt install -y -qq python3-open3d ; else python3 -m pip install open3d ; fi

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/snp \
  source /opt/tesseract_ros2/install/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args \
  && rm -rf build log
