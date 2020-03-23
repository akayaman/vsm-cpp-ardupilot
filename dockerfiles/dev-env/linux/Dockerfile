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
        libssl-dev \
        automake \
        autoconf \
        pkg-config \
        && \
# google test
    cd /tmp \
        && \
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
    make install \
        && \
    ldconfig \
        && \
# install protobuf
    cd /tmp \
        && \
    git clone https://github.com/protocolbuffers/protobuf.git \
        && \
    cd protobuf \
        && \
    git checkout v2.6.1 \
        && \
    git submodule update --init --recursive \
        && \
    sed -e s/"curl http:\/\/googletest.googlecode.com\/files\/gtest-1.5.0.tar.bz2 | tar jx"/"curl -L https:\/\/github.com\/google\/googletest\/archive\/release-1.5.0.tar.gz | tar zx"/ -i ./autogen.sh \
        && \
    sed -e s/"mv gtest-1.5.0 gtest"/"mv googletest-release-1.5.0 gtest"/ -i ./autogen.sh \
        && \
    ./autogen.sh \
        && \
    ./configure \
        && \
    make \
        && \
    make install \
        && \
    ldconfig \
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
        -DPROTOBUF_INSTALL_DIR=/usr/local/bin/ \
        -DPROTOBUF_LIBRARY=/usr/local/lib/ \
        -DPROTOBUF_INCLUDE_DIR=/usr/local/include/ \
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
# Cleanup
    cd /tmp \
        && \
    rm -rf * \
        && \
    apt remove -y \
        automake \
        autoconf \
        pkg-config \
        && \
    apt clean \
        && \
    rm -rf /var/lib/apt/lists/* \
        && \
# termination
    true

