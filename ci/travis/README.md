# Local development

Simulate a Travis build

```bash
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
```

Debug inside the container

```bash
docker run -it --rm -v "${TRAVIS_BUILD_DIR}/..:/opt/workspace/src" \
    --network host --name=osrf_ros2_nightly --privileged osrf/ros2:nightly \
    /bin/bash
```