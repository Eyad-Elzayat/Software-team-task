#!/usr/bin/env bash
# Run inside WSL (Ubuntu-22.04):   bash check_setup.sh [workspace]     default: ~/ros2_ws
# 1) files in place  2) colcon build  3) launch + topic wiring  4) automatic bench + failsafe test
WS=${1:-$HOME/ros2_ws}
T=$WS/src/thruster_ctl; R=$WS/src/rov_link
PASS=0; FAIL=0
ok()  { echo "  [ OK ] $1"; PASS=$((PASS+1)); }
bad() { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
chk() { if eval "$2"; then ok "$1"; else bad "$1"; fi; }

source /opt/ros/humble/setup.bash

echo "1) Files in place"
for f in joy_to_pilot mode_manager tam_mixer slew_limiter display_node; do
  chk "thruster_ctl/$f.py" "[ -f $T/thruster_ctl/$f.py ]"; done
for f in udp_bridge sensor_sim; do chk "rov_link/$f.py" "[ -f $R/rov_link/$f.py ]"; done
chk "launch/full.launch.py" "[ -f $T/launch/full.launch.py ]"
for e in joy_to_pilot mode_manager tam_mixer slew_limiter; do
  chk "setup.py entry point: $e" "grep -q '$e = thruster_ctl' $T/setup.py"; done
chk "setup.py installs the launch folder" "grep -q launch $T/setup.py"
chk "udp_bridge is the silent-on-timeout version" "grep -q publish_neutral_on_timeout $R/rov_link/udp_bridge.py"
chk "sensor_sim is the closed-loop version" "grep -q rov_power $R/rov_link/sensor_sim.py"
chk "mode_manager reads the Float32MultiArray imu" "grep -q Float32MultiArray $T/thruster_ctl/mode_manager.py"

echo "2) Build"
cd "$WS" || exit 1
colcon build --packages-select thruster_ctl rov_link > /tmp/build.log 2>&1
chk "colcon build (log: /tmp/build.log)" "[ \$? -eq 0 ]"
source "$WS/install/setup.bash"
for e in joy_to_pilot mode_manager tam_mixer slew_limiter thruster_display; do
  chk "executable $e" "ros2 pkg executables thruster_ctl | grep -q $e"; done
for e in udp_bridge sensor_sim; do
  chk "executable $e" "ros2 pkg executables rov_link | grep -q $e"; done

echo "3) Launch and wiring (5 s)"
if [ -n "$(ros2 node list 2>/dev/null)" ]; then
  echo "  [FAIL] ROS nodes are ALREADY running (another launch in a different terminal, or leftovers)."
  echo "         Close those terminals, or clean up with:"
  echo "           pkill -f 'ros2 launch'; pkill -f rov_ws/install; pkill -f 'topic pub'; ros2 daemon stop"
  echo "         then run this script again."
  exit 1
fi
ros2 launch thruster_ctl full.launch.py > /tmp/launch.log 2>&1 &
LP=$!
trap 'kill -INT $LP 2>/dev/null; kill $PP $PP2 2>/dev/null' EXIT
sleep 5
for n in udp_bridge sensor_sim joy_to_pilot mode_manager tam_mixer slew_limiter; do
  chk "node /$n running" "ros2 node list | grep -qx /$n"; done
for t in /pilot_cmd /cmd_wrench /thruster_cmd /rov_power /mode /armed /imu /depth; do
  chk "topic $t exists" "ros2 topic list | grep -qx $t"; done
chk "exactly ONE writer on /cmd_wrench" "ros2 topic info /cmd_wrench | grep -q 'Publisher count: 1'"
chk "exactly ONE writer on /rov_power (old thruster_mixer must be stopped)" "ros2 topic info /rov_power | grep -q 'Publisher count: 1'"

echo "4) Bench + failsafe test (fake pilot, no gamepad needed)"
ros2 topic pub -r 20 /pilot_cmd geometry_msgs/msg/Twist "{}" > /dev/null 2>&1 & PP=$!
sleep 1.5
chk "/arm accepted with centred sticks" "ros2 service call /arm std_srvs/srv/Trigger | grep -q 'success=True'"
# 'ros2 topic pub' needs ~1 s to start, longer than the 0.4 s watchdog, so the second
# publisher starts WHILE the first still runs, and only then do we stop the first.
ros2 topic pub -r 20 /pilot_cmd geometry_msgs/msg/Twist "{linear: {x: 1.0}}" > /dev/null 2>&1 & PP2=$!
sleep 2
kill -INT $PP
sleep 2
chk "mode is MANUAL"            "timeout 5 ros2 topic echo /mode --once | grep -q MANUAL"
chk "thrusters ramped up on surge" "timeout 5 ros2 topic echo /rov_power --once | grep -qE '^- -?[1-9]'"
kill -INT $PP2; sleep 1.2         # heartbeat stops = pretend the topside cable was pulled
chk "WATCHDOG: mode fell to DISARMED" "timeout 5 ros2 topic echo /mode --once | grep -q DISARMED"
chk "WATCHDOG: all thrusters are zero" "! timeout 5 ros2 topic echo /rov_power --once | grep -qE '^- -?[1-9]'"

echo; echo "Result: $PASS passed, $FAIL failed"
[ $FAIL -eq 0 ] && echo "All good. Next: real gamepad test with topside.py." || echo "Look at the first [FAIL] above; logs: /tmp/build.log, /tmp/launch.log"
