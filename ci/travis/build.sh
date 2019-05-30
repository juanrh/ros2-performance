#!/bin/bash
set -ex

docker pull osrf/ros2:nightly
docker run -it --rm -v "${TRAVIS_BUILD_DIR}/..:/opt/workspace/src" \
    --network host --name=osrf_ros2_nightly --privileged osrf/ros2:nightly \
    -e SCRIPT="${SCRIPT}" \
    -e MSG_TYPES="${MSG_TYPES}" \
    -e MAX_PUBLISHERS="${MAX_PUBLISHERS}" \
    -e MAX_SUBSCRIBERS="${MAX_SUBSCRIBERS}" \
    -e PUBLISH_FREQUENCIES="${PUBLISH_FREQUENCIES}" \
    -e DURATION="${DURATION}" \
    -e NUM_EXPERIMENTS="${NUM_EXPERIMENTS}" \
    -e MAX_SERVICES="${MAX_SERVICES}" \
    -e MAX_CLIENTS="${MAX_CLIENTS}" \
    -e REQUEST_FREQUENCIES="${REQUEST_FREQUENCIES}" \
    /bin/bash -c "/opt/workspace/src/$(basename ${TRAVIS_BUILD_DIR})/ci/travis/docker_build.sh"
