vsm-cpp-ardupilot
===========

VSM for Ardupilot autopilot implemented using VSM-SDK of [Universal ground Control Software](http://www.ugcs.com/ "UgCS").

# Preparation

```
git clone --recursive git@github.com:sensyn-robotics/vsm-cpp-ardupilot.git
cd vsm-cpp-ardupilot
```

# Build the development environment as a Docker image

At first, you build the development environment as a Docker image.
The Docker image include some tools like gcc or cmake and VSM SDK.

```shell
make ci_image
```

# Build VSM

## for Linux

### Run the Docker container as the development environment

```shell
docker-compose run --rm linux-dev bash
```

### Generate an executable binary

```shell
./linux/build.sh
```

Then, you would get `build/linux/vsm-ardupilot`.

### Run VSM

```shell
docker-compose up -d
```

## for Windows

### Run the Docker container as the development environment

```shell
docker-compose run --rm windows-dev bash
```

### Generate an executable binary

```shell
./windows/build.sh
```

Then, you would get `build/windows/vsm-ardupilot.exe`.

### C++ development with Visual Studio Code

1. Run the following commands in your host OS.

```shell
rm -rf build/include
mkdir -p build/include/system build/include/vsm
docker-compose run --rm --name windows-dev windows-dev \
    sh -c "tar zcvfh vsm-sdk.tgz /opt/vsm-sdk/vsm-sdk/include \
    && tar zcvfh system.tgz /usr/x86_64-w64-mingw32/include"
pushd build/include/vsm
tar zxvf ../../../vsm-sdk.tgz
mv opt/vsm-sdk/vsm-sdk/include/* .
mv generated/* .
mv google/* .
rm -rf opt generated google
popd
pushd build/include/system/
tar zxvf ../../../system.tgz
mv usr/x86_64-w64-mingw32/include/* .
rm -rf usr
popd
rm -f system.tgz vsm-sdk.tgz
```

2. Append the following lines into `configurations` > `includePath` in `.vscode/c_cpp_properties.json`.

```json
"${workspaceFolder}/build/include/vsm",
"${workspaceFolder}/build/include/system"
```

# Update git sub-modules

```
git submodule foreach git pull origin master
```

# The links to MAVLink

* [Messages](https://mavlink.io/en/messages/common.html)
* [Mission Protocol](https://mavlink.io/en/services/mission.html)
* [Parameter Protocol](https://mavlink.io/en/services/parameter.html)
* [Packet Serialization](https://mavlink.io/en/guide/serialization.html)

