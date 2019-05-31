# Local development

## Simulate a Travis build

```bash
export TRAVIS_BUILD_DIR=$(pwd)/src/ros2-performance
export ROS2_SDK_INSTALL_PATH="/opt/ros/dashing"
export ROS2_PERFORMANCE_TEST_INSTALL_PATH="/opt/workspace/install/"
export MERGE_INSTALL=true

export MSG_TYPES=10b
export MAX_PUBLISHERS=1
export MAX_SUBSCRIBERS=5
export PUBLISH_FREQUENCIES=100
export DURATION=10
export NUM_EXPERIMENTS=2
export SCRIPT=pub_sub_ros2.sh
... # any additional env vars used by the container

rm -rf src/rmw_dps/
src/ros2-performance/ci/travis/build.sh
```

## Debug inside the container

```bash
export TRAVIS_BUILD_DIR=$(pwd)/src/ros2-performance
docker run -it --rm -v "${TRAVIS_BUILD_DIR}/..:/opt/workspace/src" \
    --network host --name=osrf_ros2_nightly_debug --privileged osrf/ros2:nightly \
    /bin/bash
# run additional shells in the same container
docker container exec -it osrf_ros2_nightly_debug bash
```
