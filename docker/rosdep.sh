#!/bin/bash
set -e
if [ "$PWD" != "/" ];
then
  sed  -i "8,10000d" docker/rosdep.sh
  rosdep install --simulate --reinstall --from-paths src --ignore-src --reinstall | sed 's/sudo -H apt-get install/apt-get install -y/g' >> docker/rosdep.sh
fi
#[apt] Installation commands:
  apt-get install -y qtbase5-dev
  apt-get install -y libqt5core5a
  apt-get install -y libqt5gui5
  apt-get install -y libqt5widgets5
  apt-get install -y libpcl-dev
  apt-get install -y libeigen3-dev
  apt-get install -y libgeographic-dev
  apt-get install -y libasio-dev
  apt-get install -y libboost-all-dev
  apt-get install -y libpcap0.8-dev
  apt-get install -y libboost-filesystem-dev
  apt-get install -y libyaml-cpp-dev
  apt-get install -y libfmt-dev
  apt-get install -y python3-tornado
  apt-get install -y python3-bson
  apt-get install -y ros-foxy-octomap
  apt-get install -y libboost-dev
  apt-get install -y libnl-3-dev
  apt-get install -y libnl-genl-3-dev
  apt-get install -y sysstat
  apt-get install -y ntpdate
  apt-get install -y python3-rospkg
  apt-get install -y python3-pytest
  apt-get install -y wget
  apt-get install -y libopencv-dev
  apt-get install -y python3-twisted
  apt-get install -y libcrypto++-dev
  apt-get install -y libcpprest-dev
  apt-get install -y libssl-dev
  apt-get install -y python3-rtree
  apt-get install -y libtbb-dev
  apt-get install -y python3-pil
  apt-get install -y libgoogle-glog-dev
  apt-get install -y python3-termcolor
  apt-get install -y libpugixml-dev
  apt-get install -y git
  apt-get install -y libapr1-dev
  apt-get install -y libaprutil1-dev
