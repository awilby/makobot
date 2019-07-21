FROM nvidia/cudagl:9.2-devel-ubuntu16.04
RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get install -y lsb-core

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install -y ros-kinetic-perception

# Install build dependencies
RUN apt-get update && apt-get install -y \
      libpcl-dev \
      libopencv-dev \
      git \
      cmake \
      libeigen3-dev \
      software-properties-common

RUN apt-get install -y ros-kinetic-pointgrey-camera-driver
