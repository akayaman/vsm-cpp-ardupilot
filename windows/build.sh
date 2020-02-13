#!/bin/sh -eux

SRC_DIR=$(pwd)
mkdir -p build/windows
cd build/windows
rm -rf *
cmake \
  -DCMAKE_TOOLCHAIN_FILE=${SRC_DIR}/toolchain.mingw.cmake \
  -DCMAKE_INSTALL_PREFIX=/opt/vsm-sdk \
  -DUGCS_INSTALL_DIR=/opt/vsm-sdk \
  -DVSM_SDK_DIR=/opt/vsm-sdk/vsm-sdk/ \
  -DCOMMON_SOURCES="../vsm-cpp-common" \
  -DCMAKE_CXX_FLAGS="-Wno-error=implicit-fallthrough -Wno-error=old-style-cast -Wno-error=unused-parameter" \
  -DCMAKE_BUILD_TYPE=Release \
  -G"Unix Makefiles" \
  ${SRC_DIR}
sed -i -r 's/ -l(bfd|iberty|z)//g' CMakeFiles/vsm-ardupilot.dir/linklibs.rsp
echo " -I/usr/local/include" >> CMakeFiles/vsm-ardupilot.dir/includes_CXX.rsp
cmake --build .
