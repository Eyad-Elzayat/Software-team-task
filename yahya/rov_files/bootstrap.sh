#!/usr/bin/env bash
# Builds a FRESH ROS 2 workspace (~/rov_ws) with both packages from scratch.
# Run inside WSL Ubuntu-22.04:   bash bootstrap.sh <folder-with-the-downloaded-files>
# Example:                       bash /mnt/c/Users/IT/rov_files/bootstrap.sh /mnt/c/Users/IT/rov_files
SRC=${1:-.}
WS=$HOME/rov_ws

die() { echo "ERROR: $1"; exit 1; }

[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found - run this inside the Ubuntu-22.04 WSL distro"
source /opt/ros/humble/setup.bash

echo "== 1) Checking the downloaded files in $SRC"
for f in joy_to_pilot mode_manager tam_mixer slew_limiter display_node sensor_sim udp_bridge; do
  [ -f "$SRC/$f.py" ] || die "missing $SRC/$f.py"
done
[ -f "$SRC/full.launch.py" ] || die "missing $SRC/full.launch.py"

echo "== 2) Checking numpy (needed by tam_mixer and slew_limiter)"
python3 -c "import numpy" 2>/dev/null || sudo apt install -y python3-numpy

echo "== 3) Creating workspace $WS"
[ -e "$WS" ] && die "$WS already exists - delete it (rm -rf $WS) or edit WS= at the top of this script"
mkdir -p "$WS/src" && cd "$WS/src" || die "cannot create workspace"

echo "== 4) Creating the two packages"
ros2 pkg create --build-type ament_python --license MIT thruster_ctl \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs std_srvs launch_ros > /dev/null || die "pkg create thruster_ctl failed"
ros2 pkg create --build-type ament_python --license MIT rov_link \
  --dependencies rclpy std_msgs sensor_msgs > /dev/null || die "pkg create rov_link failed"

echo "== 5) Copying your nodes into them"
for f in joy_to_pilot mode_manager tam_mixer slew_limiter display_node; do
  cp "$SRC/$f.py" thruster_ctl/thruster_ctl/
done
mkdir -p thruster_ctl/launch && cp "$SRC/full.launch.py" thruster_ctl/launch/
for f in udp_bridge sensor_sim; do cp "$SRC/$f.py" rov_link/rov_link/; done

echo "== 6) Writing the two setup.py files (these register the 'ros2 run' commands)"
cat > thruster_ctl/setup.py <<'PY'
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'thruster_ctl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rov',
    maintainer_email='rov@example.com',
    description='ROV control stack: pilot input, mode manager, TAM, soft-start, display',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joy_to_pilot = thruster_ctl.joy_to_pilot:main',
            'mode_manager = thruster_ctl.mode_manager:main',
            'tam_mixer = thruster_ctl.tam_mixer:main',
            'slew_limiter = thruster_ctl.slew_limiter:main',
            'thruster_display = thruster_ctl.display_node:main',
        ],
    },
)
PY
cat > rov_link/setup.py <<'PY'
from setuptools import find_packages, setup

package_name = 'rov_link'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rov',
    maintainer_email='rov@example.com',
    description='UDP bridge to the topside and the sensor simulator',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_bridge = rov_link.udp_bridge:main',
            'sensor_sim = rov_link.sensor_sim:main',
        ],
    },
)
PY

echo "== 7) Building (this takes about a minute)"
cd "$WS" && colcon build > /tmp/build.log 2>&1 || { tail -n 20 /tmp/build.log; die "build failed, full log: /tmp/build.log"; }

echo
echo "DONE. Workspace: $WS"
echo "Next, in EVERY new terminal:   source $WS/install/setup.bash"
echo "Verify everything:             bash $SRC/check_setup.sh $WS"
