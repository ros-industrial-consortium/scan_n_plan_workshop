ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract_ros2:${TAG}

# Redeclare TAG argument for use later
ARG TAG

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Update and install Open3d
RUN apt update -y -qq \
  && if [[ "${TAG}" < "jazzy" ]] ; then \
    apt install -y -qq python3-open3d ; \
  else \
    apt install -y -qq python3-pip \
    && python3 -m pip install open3d --break-system-packages ; \
  fi

# Build the workspace and install the build artifacts
# Use a tmpfs mount for the workspace so as not to unnecessarily copy files into the final image
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/snp
ARG INSTALL_DIR=/opt/snp
RUN mkdir -p ${INSTALL_DIR}

RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/snp \
  source /opt/tesseract_ros2/install/setup.bash \
  && cd ${WORKSPACE_DIR} \
  && vcs import src < src/snp/dependencies.repos --shallow \
  && rosdep install \
    --from-paths src \
    -iry \
    --skip-keys "libvtk python3-open3d" \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install ${INSTALL_DIR}
