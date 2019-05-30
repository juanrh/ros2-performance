#!/bin/bash
set -ex

#####################
# Local development 
#####################

export TRAVIS_BUILD_DIR=$(pwd)/src/ros2-performance
export MSG_TYPES=10b
export MAX_PUBLISHERS=1
export MAX_SUBSCRIBERS=5
export PUBLISH_FREQUENCIES=100
export DURATION=10
export NUM_EXPERIMENTS=2
export SCRIPT=pub_sub_ros2.sh 

rm -rf src/rmw_dps/
src/ros2-performance/ci/travis/build.sh

# Debug inside the container
docker run -it --rm -v "${TRAVIS_BUILD_DIR}/..:/opt/workspace/src" \
    --network host --name=osrf_ros2_nightly --privileged osrf/ros2:nightly \
    /bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

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
