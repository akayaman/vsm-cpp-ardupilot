FROM ubuntu:18.04

RUN \
# install tools
    apt update -y \
        && \
    apt install -y \
        cmake \
        python2.7 \
        git \
        curl \
        iputils-ping \
        net-tools \
        libtool \
        unzip \
        wget \
        bison \
        flex \
        && \
    ln -s /usr/bin/python2.7 /usr/bin/python \
        && \
# Cleanup
    apt clean \
        && \
    rm -rf /var/lib/apt/lists/* \
# termination
        && \
    true

WORKDIR /home/build
ENV LANG=C.UTF-8
