#!/bin/sh -eux

SRC_DIR=$(pwd)
mkdir -p build/linux
cd build/linux
rm -rf *
cmake \
  -DCMAKE_INSTALL_PREFIX=/opt/vsm-sdk/linux \
  -DUGCS_INSTALL_DIR=/opt/vsm-sdk/linux \
  -DVSM_SDK_DIR=/opt/vsm-sdk/linux/opt/vsm-sdk/ \
  -DCOMMON_SOURCES="../vsm-cpp-common" \
  -DCMAKE_CXX_FLAGS="-Wno-error=implicit-fallthrough -Wno-error=old-style-cast -Wno-error=unused-parameter -g3" \
  -G"Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  ${SRC_DIR}
cmake --build .
