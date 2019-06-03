#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

function initialize_environment {
    apt-get update
    source /opt/ros/dashing/setup.bash
    pip3 install 'jinja2>=2.10' --user
    MATPLOTLIB_RC="${HOME}/.config/matplotlib/matplotlibrc"
    mkdir -p "$(dirname ${MATPLOTLIB_RC})"
    echo "backend: Agg" > "${MATPLOTLIB_RC}"

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
    colcon build --packages-up-to performance_test benchmark --merge-install
    echo "Done setting up ros2-performance"
}

function run_benchmarks {
    TEST_DURATION=60
    IPC='on'
    BENCHMARK_IDS='sierra_nevada mont_blanc'

    source install/local_setup.bash
    pushd install/lib/benchmark
    git clone --branch=${GH_PAGES_BRANCH} https://${GH_TOKEN}@github.com/${TRAVIS_REPO_SLUG}.git ${GH_PAGES_BRANCH}
    PERFORMANCES_ROOT="$(pwd)/../../../src/ros2-performance/performances"

    for BENCHMARK_ID in ${BENCHMARK_IDS}
    do
        ${BENCHMARK_SETUP}
        echo "--------------------------------------------------"
        echo "Running benchmark [${BENCHMARK_ID}]"
        echo "--------------------------------------------------"
        RMW_IMPLEMENTATION=rmw_dps_cpp ./benchmark "topology/${BENCHMARK_ID}.json" -t ${TEST_DURATION} --ipc ${IPC}
        echo "Done running benchmark [${BENCHMARK_ID}]"
        echo

        RESULTS_ID="${CURRENT_DATE_FORMAT}/${BENCHMARK_ID}"
        RESULTS_ROOT="${GH_PAGES_BRANCH}/results/${RESULTS_ID}"
        mkdir -p "${RESULTS_ROOT}"
        # copy results data 
        tar cvzf "${RESULTS_ROOT}/log.tgz" log
        # generate plots
        mkdir -p "${RESULTS_ROOT}/plots"
        python3 "${PERFORMANCES_ROOT}/performance_test/scripts/plot_scripts/benchmark_app_evaluation.py" \
            --target "${PERFORMANCES_ROOT}/benchmark/performance_target/sierra_nevada_rpi3.json" \
            --resources "log/resources.txt" \
            --latency "log/latency_total.txt" \
            --plot-filename "${RESULTS_ROOT}/plots/benchmark_app_evaluation.svg"
        echo "generated ${RESULTS_ROOT}/plots/benchmark_app_evaluation.svg"
        python3 "${PERFORMANCES_ROOT}/performance_test/scripts/plot_scripts/cpu_ram_plot.py" \
            "log/resources.txt" \
            --target "${PERFORMANCES_ROOT}/benchmark/performance_target/sierra_nevada_rpi3.json" \
            --x time --y cpu --y2 rss \
            --plot-filename "${RESULTS_ROOT}/plots/cpu_ram_plot.svg"
        echo "generated ${RESULTS_ROOT}/plots/cpu_ram_plot.svg"
        echo
    done
    popd
    echo "Done running experiments"
}

function publish_benchmark_results {
    pushd "install/lib/benchmark/${GH_PAGES_BRANCH}"

    # Upload the results for all benchmarks run on this job
    RESULTS_ID="${CURRENT_DATE_FORMAT}"
    RESULTS_ROOT="results/${RESULTS_ID}"
    echo "Publishing benchmark results for [${RESULTS_ID}]"
    python3 "${SCRIPT_DIR}/write_results_index.py" "${RESULTS_ROOT}"
    echo "- [Benchmark ${RESULTS_ID}](results/${RESULTS_ID}) for [Travis build ${TRAVIS_BUILD_NUMBER}](${TRAVIS_BUILD_WEB_URL}) and [Travis job ${TRAVIS_JOB_NUMBER}](${TRAVIS_JOB_WEB_URL})" >> README.md
    git add .
    git commit -m "Upload ${RESULTS_ROOT} for Travis build ${TRAVIS_BUILD_NUMBER} and job ${TRAVIS_JOB_NUMBER}"
    git push origin ${GH_PAGES_BRANCH}
    popd
    echo "Done publishing benchmark results for [${RESULTS_ID}]"
}

initialize_environment
setup_git

mkdir -p /opt/workspace/src
pushd /opt/workspace
setup_rmw_dps
setup_ros2_performance
run_benchmarks
publish_benchmark_results
popd
