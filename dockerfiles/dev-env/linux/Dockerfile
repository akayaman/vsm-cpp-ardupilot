ARG BASE_IMAGE_MD5=latest
FROM nakayamaatsushi/fq1150-vsm-build-base:${BASE_IMAGE_MD5}

RUN \
# install tools
    apt update -y \
        && \
    apt install -y \
        g++ \
        gcc \
        gnulib \
        libboost-dev \
        libboost-test-dev \
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-thread-dev \
        libevent-dev \
        zlib1g-dev \
        libtool \
        libssl-dev \
        wget \
        automake \
        bison \
        flex \
        pkg-config \
        && \
# clone VSM C++ SDK
    cd /tmp \
        && \
    git clone https://github.com/ugcs/vsm-cpp-sdk.git \
        && \
    mkdir -p vsm-cpp-sdk/build \
        && \
    cd vsm-cpp-sdk/build \
        && \
# generate Makefile for Linux.
    cmake  \
        -DUGCS_INSTALL_DIR=/opt/vsm-sdk/linux \
        -DCMAKE_INSTALL_PREFIX=/opt/vsm-sdk/linux \
        -DCMAKE_BUILD_TYPE=Release \
        -G"Unix Makefiles" \
        -DCMAKE_CXX_FLAGS='-Wno-error=implicit-fallthrough' \
        .. \
        && \
# build binaries and install them.
    cmake \
        --build .  \
        -- install  \
        -j1 \
        && \
# google test
    wget https://github.com/google/googletest/archive/release-1.8.1.tar.gz \
        && \
    tar -xvf release-1.8.1.tar.gz \
        && \
    cd googletest-release-1.8.1/ \
        && \
    mkdir build \
        && \
    cd build \
        && \
    cmake .. \
        && \
    make \
        && \
    mv ../googletest/include/gtest/ /usr/local/include/ \
        && \
    mv ../googlemock/include/gmock/ /usr/local/include/ \
        && \
    mv googlemock /usr/lib/ \
        && \
# Cleanup
    cd /tmp \
        && \
    rm -rf * \
        && \
    apt remove -y \
        automake \
        pkg-config \
        && \
    apt clean \
        && \
    rm -rf /var/lib/apt/lists/* \
        && \
# termination
    true

