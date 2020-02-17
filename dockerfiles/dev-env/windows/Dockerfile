ARG BASE_IMAGE_MD5=latest
ARG LINUX_IMAGE_MD5=latest

FROM nakayamaatsushi/fq1150-vsm-build-linux:${LINUX_IMAGE_MD5} AS linux-image
FROM nakayamaatsushi/fq1150-vsm-build-base:${BASE_IMAGE_MD5}

COPY toolchain.mingw.cmake /tmp
COPY --from=linux-image /opt/vsm-sdk/linux/opt/vsm-sdk/share/tools/protobuf_compiler /usr/local/bin/

RUN \
# install tools
    apt update -y \
        && \
    apt install -y \
        binutils-mingw-w64-x86-64 \
        mingw-w64-common \
        mingw-w64-x86-64-dev \
        mingw-w64-tools \
        gcc-mingw-w64-base \
        gcc-mingw-w64-x86-64 \
        g++-mingw-w64-x86-64 \
        win-iconv-mingw-w64-dev \
        wget \
        gcc \
        automake \
        bison \
        flex \
        pkg-config \
        && \
# configure the settings of the thread model to the POSIX one.
    update-alternatives --set x86_64-w64-mingw32-g++ /usr/bin/x86_64-w64-mingw32-g++-posix \
        && \
    update-alternatives --set x86_64-w64-mingw32-gcc /usr/bin/x86_64-w64-mingw32-gcc-posix \
        && \
# compatibility fix for the difference of latter-cases.
    ln -s /usr/x86_64-w64-mingw32/include/userenv.h /usr/x86_64-w64-mingw32/include/Userenv.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/lib/libuserenv.a /usr/x86_64-w64-mingw32/lib/libUserenv.a \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/winsock2.h /usr/x86_64-w64-mingw32/include/Winsock2.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/winsock2.h /usr/x86_64-w64-mingw32/include/WinSock2.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/shlwapi.h /usr/x86_64-w64-mingw32/include/Shlwapi.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/windows.h /usr/x86_64-w64-mingw32/include/Windows.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/accctrl.h /usr/x86_64-w64-mingw32/include/AccCtrl.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/aclapi.h /usr/x86_64-w64-mingw32/include/Aclapi.h \
        && \
    ln -s /usr/x86_64-w64-mingw32/include/ws2tcpip.h /usr/x86_64-w64-mingw32/include/WS2tcpip.h \
        && \
# install zlib for Windows.
    cd /tmp \
        && \
    wget http://zlib.net/zlib-1.2.11.tar.gz -O- | tar xfz - \
        && \
    cd zlib* \
        && \
    sed -e s/"PREFIX ="/"PREFIX = x86_64-w64-mingw32-"/ -i win32/Makefile.gcc \
        && \
    make -f win32/Makefile.gcc \
        && \
    BINARY_PATH=/usr/x86_64-w64-mingw32/bin \
    INCLUDE_PATH=/usr/x86_64-w64-mingw32/include \
    LIBRARY_PATH=/usr/x86_64-w64-mingw32/lib \
    make -f win32/Makefile.gcc install \
        && \
# install OpenSSL for Windows.
    cd /tmp \
        && \
    wget https://www.openssl.org/source/openssl-1.0.2.tar.gz -O- | tar xfz - \
        && \
    cd openssl* \
        && \
    CROSS_COMPILE=x86_64-w64-mingw32- \
        ./Configure \
            -DHAVE_STRUCT_TIMESPEC \
            -L/usr/x86_64-w64-mingw32/lib \
            -lz \
            -lws2_32 \
            --prefix=/usr/x86_64-w64-mingw32 \
            zlib \
            mingw64 \
            shared \
            no-asm \
        && \
    make \
        && \
    make install \
        && \
# install libevent for Windows.
    cd /tmp \
        && \
    git clone https://github.com/libevent/libevent.git \
        && \
    mkdir -p libevent/build \
        && \
    cd libevent \
        && \
    git checkout -b release-2.1.8-stable \
        && \
    cd build \
        && \
    cmake  \
        -DCMAKE_TOOLCHAIN_FILE=/tmp/toolchain.mingw.cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/x86_64-w64-mingw32 \
        -DCMAKE_C_FLAGS="-Wall -Wswitch -fno-strict-aliasing -DEVENT__HAVE_STRTOK_R" \
        -G"Unix Makefiles" \
        .. \
        && \
    make \
        && \
    make install \
        && \
#install boost library  for Windows.
    cd /tmp \
        && \
    wget https://dl.bintray.com/boostorg/release/1.67.0/source/boost_1_67_0.tar.gz \
        && \
    tar xzfp boost_1_67_0.tar.gz \
        && \
    cd boost_1_67_0 \
        && \
    echo "using gcc : : x86_64-w64-mingw32-g++ ;" > user-config.jam \
        && \
    ./bootstrap.sh --without-libraries=python \
        && \
    ./b2 \
        abi=ms \
        address-model=64 \
        architecture=x86 \
        binary-format=pe \
        threading=multi \
        toolset=gcc \
        link=shared \
        target-os=windows \
        variant=release \
        --without-mpi \
        --without-python \
        --user-config=user-config.jam \
        && \
    ./bjam install --prefix=/usr/x86_64-w64-mingw32 \
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
# generate Makefile for Windows.
    cmake  \
        -DCMAKE_TOOLCHAIN_FILE=/tmp/toolchain.mingw.cmake \
        -DPROTOBUF_PROTOC_BINARY=/opt/vsm-sdk/linux/share/tools/protobuf_compiler \
        -DUGCS_INSTALL_DIR=/opt/vsm-sdk \
        -DCMAKE_INSTALL_PREFIX=/opt/vsm-sdk \
        -DCMAKE_BUILD_TYPE=Release \
        -G"Unix Makefiles" \
        -DCMAKE_CXX_FLAGS='-Wno-error=implicit-fallthrough' \
        .. \
        && \
# build binaries but fails since wrong executable protobuf_compiler.
    (cmake \
        --build .  \
        -- install  \
        -j8 \
    || true) \
        && \
    rm -f ./protobuf/protobuf_compiler.exe \
        && \
    ln -s /usr/local/bin/protobuf_compiler ./protobuf/protobuf_compiler.exe \
        && \
    touch ./protobuf/protobuf_compiler.exe \
        && \
# retry to build binaries and install them.
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
        gcc \
        automake \
        pkg-config \
        && \
    apt clean \
        && \
    rm -rf /var/lib/apt/lists/* \
        && \
# termination
    true
