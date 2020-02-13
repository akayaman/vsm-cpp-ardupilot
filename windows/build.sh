#!/bin/sh -eux

SRC_DIR=$(pwd)
mkdir -p build/windows
cd build/windows

cp /usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll .
cp /usr/lib/gcc/x86_64-w64-mingw32/7.3-posix/libstdc++-6.dll .
cp /usr/lib/gcc/x86_64-w64-mingw32/7.3-posix/libgcc_s_seh-1.dll .
cp /usr/x86_64-w64-mingw32/bin/ssleay32.dll .
cp /usr/x86_64-w64-mingw32/bin/libeay32.dll .

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
