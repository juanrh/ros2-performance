#!/bin/bash
set -e

function initialize_environment {
    apt-get update
    source /opt/ros/dashing/setup.bash

    export ROS2_SDK_INSTALL_PATH="/opt/ros/dashing"
    export ROS2_PERFORMANCE_TEST_INSTALL_PATH="/opt/workspace/install/"
    export MERGE_INSTALL=true
}

function setup_git {
    git config --global user.name nobody
    git config --global user.email noreply@osrfoundation.org
}

function setup_rmw_dps {
    echo "Setting up DPS RMW"
    pushd src
    git clone https://github.com/ros2/rmw_dps.git
    popd
    rosdep update && rosdep install --from-paths src/rmw_dps --ignore-src -r -y
    # For ros2-performance all workspaces or none have to be build with merge install, and
    # ROS 2 was built with merge install in this image
    colcon build --packages-up-to rmw_dps_cpp --merge-install
    colcon test --packages-select rmw_dps_cpp --merge-install
    colcon test-result --verbose
    
    source install/setup.bash
    echo "Done setting up DPS RMW"
}

function setup_ros2_performance {
    echo "Setting up ros2-performance"
    echo "WORKAROUND: disabling wait_for_discovery for performance experiments until is is supported by RMW DPS"
    sed -i 's@this->wait_discovery@//this->wait_discovery@g' src/ros2-performance/performances/performance_test/src/ros2/system.cpp
    colcon build --packages-up-to performance_test_msgs performance_test --merge-install
    echo "Done setting up ros2-performance"
}

function run_experiments {
    echo "Running experiments"
    pushd src/ros2-performance/performances/performance_test
    source env.sh
    RMW_IMPLEMENTATION=rmw_dps_cpp bash -x "scripts/${SCRIPT}"
    popd
    echo "Done running experiments"
}

function publish_experiments_results {
    echo "Publishing experiments results"
    # TODO build plots
    # use PLOT_FILENAME to enable plotting to file
    echo "Done publishing experiments results"
}

initialize_environment
setup_git

mkdir -p /opt/workspace/src
pushd /opt/workspace
setup_rmw_dps
setup_ros2_performance
run_experiments
publish_experiments_results
popd
