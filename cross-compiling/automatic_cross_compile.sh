# This script is intended to be used by the Jenkins night builds.
# But it still possible to use manually setting up this variables:
#
# TARGET=[raspbian]
# ROS2_BRANCH=[ardent,bouncy,crystal,master,...]

# Check that all variables are defined before start
if [[ -z "$TARGET" || -z "$ROS2_BRANCH" ]]
then
      echo "Error: environmental variables are not defined!"
      echo "Set values to: TARGET / ROS2_BRANCH"
      exit 1
fi

THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
export BASE_DIR="$(dirname "$(pwd)")"

# Prepare cross-compiling environment
source $THIS_DIR/env.sh $TARGET

# Get sysroot
bash $THIS_DIR/sysroots/get-sysroots.sh

# Remove ROS2 old cross-compilation workspace and get a new one
cd $BASE_DIR
sudo rm -rf ros2_cc_ws
mkdir -p ros2_cc_ws/src
cd ros2_cc_ws
wget https://raw.githubusercontent.com/ros2/ros2/"$ROS2_BRANCH"/ros2.repos
vcs import src < ros2.repos

# Get HEAD of branch
HEAD=$(git ls-remote git://github.com/ros2/ros2 "$ROS2_BRANCH" | cut -c1-7)

# Create install directory for the cross-compilation results
RESULTS_DIR=$BASE_DIR/install/"$ROS2_BRANCH"_"$HEAD"_"$TARGET"
echo "Create RESULTS_DIR=$RESULTS_DIR"
mkdir -p $RESULTS_DIR

# Save the current packages versions and check if any changes
ROS2_SRCS_HEADS=$RESULTS_DIR/ros2.repos.by_commit
ROS2_SRCS_HEADS_PREV_RUN=$BASE_DIR/ros2_srcs_prev_run_"$TARGET"_"$ROS2_BRANCH".txt

vcs export --exact src > $ROS2_SRCS_HEADS

if [ -f $ROS2_SRCS_HEADS_PREV_RUN ]; then
    diff -y  $ROS2_SRCS_HEADS $ROS2_SRCS_HEADS_PREV_RUN > $RESULTS_DIR/diff_ros2_versions.txt
fi
cp -R $ROS2_SRCS_HEADS $ROS2_SRCS_HEADS_PREV_RUN


# Cross-compiling ROS2
cd $BASE_DIR
IGNORE_SCRIPT=cross-compiling/ignore_pkgs/"$ROS2_BRANCH"_ignore.sh
bash $IGNORE_SCRIPT $BASE_DIR/ros2_cc_ws
CMD="bash cross-compiling/cc_workspace.sh $BASE_DIR/ros2_cc_ws"

# If the build was succesful, copy results to store as artifact
if $CMD; then
    cp -r $BASE_DIR/ros2_cc_ws/install $RESULTS_DIR
else
    exit 1
fi
