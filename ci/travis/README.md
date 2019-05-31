# Local development

## Simulate a Travis build

```bash
export TRAVIS_BUILD_DIR=$(pwd)/src/ros2-performance
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
