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

function safe_run {
    # Runs a command redirecting it's stdout to stderr, prints the return code on the stdout
    # and always returns 0
    # Examples: 
    #   C=$(safe_run 'ls -al asdasd')
    #   C=$(safe_run 'ls -al')
    CMD="${1}"
    RETURN_CODE=0
    ${CMD} 1>&2 || RETURN_CODE=$?
    echo ${RETURN_CODE}
}

function setup_rmw_dps {
    echo "Setting up DPS RMW"
    echo

    pushd src
    git clone https://github.com/ros2/rmw_dps.git
    popd
    rosdep update && rosdep install --from-paths src/rmw_dps --ignore-src -r -y
    # For ros2-performance all workspaces or none have to be build with merge install, and
    # ROS 2 was built with merge install in this image
    colcon build --packages-up-to rmw_dps_cpp --merge-install

    echo "WORKAROUND: disabling wait_for_discovery for performance experiments until is is supported by RMW DPS"
    PATCHED_FILE='src/ros2-performance/performances/performance_test/src/ros2/system.cpp'
    sed -i 's@this->wait_discovery@//this->wait_discovery@g' "${PATCHED_FILE}"
    setup_ros2_performance
    pushd $(dirname "${PATCHED_FILE}")
    git checkout $(basename "${PATCHED_FILE}")
    popd

    echo
    echo "Done setting up DPS RMW"
}

function setup_rmw_cyclonedds {
    echo "Setting up Cyclone DDS RMW"
    echo
    export CYCLONE_DDS_ROOT='/opt/cyclonedds'
    echo "Installing Cyclone DDS at [${CYCLONE_DDS_ROOT}]"
    mkdir -p "${CYCLONE_DDS_ROOT}"
    mkdir -p "${CYCLONE_DDS_ROOT}-src"
    pushd "${CYCLONE_DDS_ROOT}-src"
    git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
    # Install dependencies. Per https://github.com/eclipse-cyclonedds/cyclonedds#building-eclipse-cyclone-dds
    # > The Java-based components are the preprocessor and a configurator tool. The run-time libraries are pure C code, so there is no need to have Java available on "target" machines.
    apt-get install maven default-jdk -y
    mkdir cyclonedds/build
    pushd cyclonedds/build
    cmake -DCMAKE_INSTALL_PREFIX="${CYCLONE_DDS_ROOT}" ../src/
    cmake --build .
    cmake --build . --target install
    popd
    popd
    # Set this if using a value for CYCLONE_DDS_ROOT that is not a standard path
    # export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH};${CYCLONE_DDS_ROOT}
    # This is required because cyclonedds is installed without using ROS, so 
    # ROS packages fail with "Cannot load library: libddsc.so.0" when using 
    # `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` if we don't set LD_LIBRARY_PATH
    # TODO modify rmw_cyclonedds to install cyclonedds properly
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CYCLONE_DDS_ROOT}/lib
    echo "Done installing Cyclone DDS"

    pushd src
    git clone https://github.com/atolab/rmw_cyclonedds.git
    popd
    rosdep update && rosdep install --from-paths src/rmw_cyclonedds --ignore-src -r -y
    # For ros2-performance all workspaces or none have to be build with merge install, and
    # ROS 2 was built with merge install in this image
    colcon build --packages-up-to rmw_cyclonedds_cpp --merge-install
    # Disabled as currently the repo has cpplint test failures
    # colcon test --packages-select rmw_cyclonedds_cpp --merge-install
    # colcon test-result --verbose

    setup_ros2_performance

    echo
    echo "Done setting up Cyclone DDS RMW"
}

function setup_ros2_performance {
    echo "Setting up ros2-performance"
    echo
    rm -rf build/performance_test* build/benchmark*
    colcon build --packages-up-to performance_test benchmark --merge-install
    echo
    echo "Done setting up ros2-performance"
}

function setup_gh_pages {
    pushd install/lib/benchmark
    git clone --branch=${GH_PAGES_BRANCH} https://${GH_TOKEN}@github.com/${TRAVIS_REPO_SLUG}.git ${GH_PAGES_BRANCH}
    export GH_PAGES_ROOT="$(pwd)/${GH_PAGES_BRANCH}"
    popd
}

function run_benchmarks {
    TEST_DURATION=60
    IPC='on'
    BENCHMARK_IDS='sierra_nevada mont_blanc'
    PERFORMANCES_ROOT="/opt/workspace/src/ros2-performance/performances"
    BENCHMARK_EXIT_CODE_FILE='log/benchmark_exit_code.txt'

    source install/local_setup.bash
    pushd install/lib/benchmark
    for BENCHMARK_ID in ${BENCHMARK_IDS}
    do
        ${BENCHMARK_SETUP}
        echo "--------------------------------------------------"
        echo "Running benchmark [${BENCHMARK_ID}]"
        echo "--------------------------------------------------"
        BENCHMARK_EXIT_CODE=$(RMW_IMPLEMENTATION=rmw_dps_cpp safe_run "./benchmark "topology/${BENCHMARK_ID}.json" -t ${TEST_DURATION} --ipc ${IPC}")
        echo ${BENCHMARK_EXIT_CODE} > ${BENCHMARK_EXIT_CODE_FILE}
        echo "Execution of benchmark [${BENCHMARK_ID}] completed with exit code [${BENCHMARK_EXIT_CODE}]"
        echo

        RESULTS_ID="${CURRENT_DATE_FORMAT}/rmw_${RMW_IMPL}/${BENCHMARK_ID}"
        RESULTS_ROOT="${GH_PAGES_ROOT}/results/${RESULTS_ID}"
        mkdir -p "${RESULTS_ROOT}"
        # copy results data 
        tar cvzf "${RESULTS_ROOT}/log.tgz" log
        cp "${BENCHMARK_EXIT_CODE_FILE}" "${RESULTS_ROOT}"
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
    pushd "${GH_PAGES_ROOT}"

    # Upload the results for all benchmarks run on this job
    RESULTS_ID="${CURRENT_DATE_FORMAT}"
    RESULTS_ROOT="results/${RESULTS_ID}"
    echo "Publishing benchmark results for [${RESULTS_ID}]"
    python3 "${SCRIPT_DIR}/write_results_index.py" "${RESULTS_ROOT}" \
        "${TRAVIS_BUILD_NUMBER}" "${TRAVIS_BUILD_WEB_URL}" "${TRAVIS_JOB_NUMBER}" "${TRAVIS_JOB_WEB_URL}"
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
setup_gh_pages

RMW_IMPLS='dps cyclonedds'
for RMW_IMPL in ${RMW_IMPLS}
do
    echo "Running benchmarks for RMW implementation [${RMW_IMPL}]"
    setup_rmw_${RMW_IMPL}
    run_benchmarks
    echo "Done running benchmarks for RMW implementation [${RMW_IMPL}]"
done

publish_benchmark_results
popd
