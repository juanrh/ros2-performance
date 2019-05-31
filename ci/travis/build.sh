#!/bin/bash
set -e

CURRENT_DATE_FORMAT="$(date +%Y-%m-%d_%H-%M-%S)_UTC"
docker image pull osrf/ros2:nightly
docker container run -it --rm -v "${TRAVIS_BUILD_DIR}/..:/opt/workspace/src" \
    -e SCRIPT="${SCRIPT}" \
    -e MSG_TYPES="${MSG_TYPES:-nil}" \
    -e MAX_PUBLISHERS="${MAX_PUBLISHERS:-nil}" \
    -e MAX_SUBSCRIBERS="${MAX_SUBSCRIBERS:-nil}" \
    -e PUBLISH_FREQUENCIES="${PUBLISH_FREQUENCIES:-nil}" \
    -e DURATION="${DURATION:-nil}" \
    -e NUM_EXPERIMENTS="${NUM_EXPERIMENTS:-nil}" \
    -e MAX_SERVICES="${MAX_SERVICES:-nil}" \
    -e MAX_CLIENTS="${MAX_CLIENTS:-nil}" \
    -e REQUEST_FREQUENCIES="${REQUEST_FREQUENCIES:-nil}" \
    -e GH_PAGES_BRANCH="${GH_PAGES_BRANCH}" \
    -e GH_TOKEN="${GH_TOKEN}" \
    -e CURRENT_DATE_FORMAT="${CURRENT_DATE_FORMAT}" \
    -e TRAVIS_REPO_SLUG="${TRAVIS_REPO_SLUG}" \
    -e TRAVIS_BUILD_NUMBER="${TRAVIS_BUILD_NUMBER}" \
    -e TRAVIS_BUILD_WEB_URL="${TRAVIS_BUILD_WEB_URL}" \
    -e TRAVIS_JOB_NUMBER="${TRAVIS_JOB_NUMBER}" \
    -e TRAVIS_JOB_WEB_URL="${TRAVIS_JOB_WEB_URL}" \
    --network host --name=osrf_ros2_nightly --privileged osrf/ros2:nightly \
    /bin/bash -c "/opt/workspace/src/$(basename ${TRAVIS_BUILD_DIR})/ci/travis/docker_build.sh"
