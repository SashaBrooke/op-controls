FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Download Linux support tools
RUN apt-get update && \
    apt-get clean &&  \
    apt-get install -y \
        build-essential \
        wget \
        curl \
        git  \
        python3 \
        python3-pip \
        vim \
        less

# Setup the dev directory
WORKDIR /home/dev

# Download and install RP2040 Toolchains
RUN apt-get -y install cmake \
        gcc-arm-none-eabi \
        libnewlib-arm-none-eabi 

# Clone and setup the RP2040 SDK
RUN git clone https://github.com/raspberrypi/pico-sdk /home/package/pico-sdk && \
     cd /home/package/pico-sdk && \
    git submodule update --init

ENV PICO_SDK_PATH=/home/package/pico-sdk

# Clone and setup NanoPB
RUN git clone https://github.com/nanopb/nanopb/ /home/package/nanopb && \
    cd /home/package/nanopb && \
    git submodule update --init && \
    pip3 install --user -r requirements.txt

ENV NANOPB_PATH=/home/package/nanopb

# Install picotool dependencies
RUN apt-get install -y libusb-1.0-0-dev

# Pre-install picotool with cmake
RUN git clone https://github.com/raspberrypi/picotool.git /home/package/picotool && \
    cd /home/package/picotool && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    cmake --install .

# Final working directory
WORKDIR /home/Desktop
