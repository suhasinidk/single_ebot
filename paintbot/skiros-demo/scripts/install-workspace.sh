#!/bin/bash

# Repository
REPO=https://github.com/Bjarne-AAU/skiros-demo.git

# Usage
function print-help
{
	echo "Usage: ./install folder ros_distro"
}

# Args parser
function check-params
{
  if [ $# -lt 2 ]; then
	  echo "Not enough arguments provided"
	  print-help
	  exit -1
  fi
}

function install-special-dependencies
{
  ./src/skiros2/skiros2/skiros2/scripts/install_fd_task_planner.sh $1
}


# import tools
SCRIPT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
. $SCRIPT/tools.sh

check-params $*
ROS=$2
if check-pkg-installed ros-$ROS-ros -eq 0; then
    echo "Ros distro $ROS not found. Install ROS and check the distro name." 1>&2
    exit -1
else
    echo "Found ROS distro $ROS."
fi
install-pkgs git python-catkin-tools python-pip
download-repo $REPO $1
cd $1
install-repo-dependencies $ROS
install-special-dependencies
add-source-to-bash "/opt/ros/$ROS/setup.bash"
build-repo
add-source-to-bash "`pwd`/devel/setup.bash"
bash
