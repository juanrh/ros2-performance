#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function initialize_environment {
    apt-get update
    source /opt/ros/dashing/setup.bash

    export ROS2_SDK_INSTALL_PATH="/opt/ros/dashing"
    export ROS2_PERFORMANCE_TEST_INSTALL_PATH="/opt/workspace/install/"
    export MERGE_INSTALL=true
}

function setup_git {
    git config --global user.name nobody
    git config --global user.email noreply@none.org
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
    # Using (mostly) default values, TODO setup suite more carefully
    
    function experiment_setup1 {
        export SCRIPT=pub_sub_ros2.sh
        export MSG_TYPES=10b 
        export MAX_PUBLISHERS=1
        export MAX_SUBSCRIBERS=5
        export PUBLISH_FREQUENCIES=100
        export DURATION=10
        export NUM_EXPERIMENTS=2
    }

    function experiment_setup2 {
        export SCRIPT=pub_sub_separate_process.sh
        export MSG_TYPES=10b 
        export MAX_PUBLISHERS=1 
        export MAX_SUBSCRIBERS=5
        export PUBLISH_FREQUENCIES=100 
        export DURATION=10
        export NUM_EXPERIMENTS=2
    }

    function experiment_setup3 {
        export SCRIPT=client_service_ros2.sh 
        export MAX_SERVICES=1 
        export MAX_CLIENTS=5 
        export REQUEST_FREQUENCIES=100
        export DURATION=10
        export NUM_EXPERIMENTS=2
    }

    function experiment_setup4 {
        export SCRIPT=only_subs.sh
        export MSG_TYPES=10b
        export MAX_PUBLISHERS=1
        export MAX_SUBSCRIBERS=5
        export DURATION=10
        export NUM_EXPERIMENTS=2
    }

    EXPERIMENT_SETUPS="experiment_setup1 experiment_setup2 experiment_setup3 experiment_setup4"
    echo "Running experiments"
    pushd src/ros2-performance/performances/performance_test
    source env.sh
    for EXPERIMENT_SETUP in ${EXPERIMENT_SETUPS}
    do
        set -x
        ${EXPERIMENT_SETUP}
        set +x
        RMW_IMPLEMENTATION=rmw_dps_cpp bash -x "scripts/${SCRIPT}"
    done    
    popd
    echo "Done running experiments"
}

function publish_experiments_results {
    # FIXME: this is broken until adapted to running multiple experiments on run_experiments

    echo "Publishing experiments results"
    pip3 install jinja2>=2.10 --user

    pushd src/ros2-performance
    git clone --branch=${GH_PAGES_BRANCH} https://${GH_TOKEN}@github.com/${TRAVIS_REPO_SLUG}.git ${GH_PAGES_BRANCH}
    SCRIPT_NAME="${SCRIPT%.*}"
    RESULTS_ID="${CURRENT_DATE_FORMAT}-${SCRIPT_NAME}"
    RESULTS_ROOT="${GH_PAGES_BRANCH}/results/${RESULTS_ID}"
    mkdir -p "${RESULTS_ROOT}"
    cp performances/performance_test/results/*/* ${RESULTS_ROOT}
    python3 "${SCRIPT_DIR}/write_results_index.py" "${RESULTS_ROOT}"
    pushd ${GH_PAGES_BRANCH}
    echo "- [Experiment ${RESULTS_ID}](results/${RESULTS_ID}) for [Travis build ${TRAVIS_BUILD_NUMBER}](${TRAVIS_BUILD_WEB_URL}) and [Travis job ${TRAVIS_JOB_NUMBER}](${TRAVIS_JOB_WEB_URL})" >> README.md
    git add .
    git commit -m "Upload ${RESULTS_ROOT} for Travis build ${TRAVIS_BUILD_NUMBER} and job ${TRAVIS_JOB_NUMBER}"
    git push origin ${GH_PAGES_BRANCH}
    popd

    # TODO build plots
    # use PLOT_FILENAME to enable plotting to file
    popd
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
