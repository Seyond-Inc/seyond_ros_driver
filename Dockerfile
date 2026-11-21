# syntax=docker/dockerfile:1
# Base: the same public image as before (ros:jazzy-perception-noble), now pinned by the digest of
# its multi-arch index as found on Docker Hub on 2026-11-21 (tag last updated 2026-11-20 06:17 UTC).
# A build from the bare tag on that day used exactly this image. Move all three Dockerfiles together.
ARG BASE_IMAGE=ros:jazzy-perception-noble@sha256:233408de3a53d864d7f5e1c8c6ff7948d126162f22221c8f0a8785ab7b03095b
# hadolint ignore=DL3006
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy

WORKDIR /opt/metrolla/seyond
COPY . .

RUN apt-get update && apt-get install -y \
    iproute2 \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Build the driver
RUN bash -c "source /opt/ros/jazzy/setup.bash && ./build.bash"

RUN chmod +x entrypoint.sh && cp entrypoint.sh /usr/local/bin/

# The environment contract (opt-in with METROLLA_DRIVER_CONTRACT=1, AGENTS.md); its files came in
# with "COPY . ." above. entrypoint.sh execs docker/contract-entrypoint.sh only when that variable
# is set; without it the image runs the released entrypoint as before. selfcheck fails the build if
# the contract cannot render or the driver binary is missing or not usable by any uid.
RUN chmod 0755 docker/contract-entrypoint.sh docker/seyond_env.py \
    && chmod -R a+rX docker \
    && python3 -I docker/seyond_env.py selfcheck
ENTRYPOINT ["entrypoint.sh"]
# Default command to run the driver
CMD ["ros2", "launch", "seyond", "start_with_config.py"]