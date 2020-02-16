#!/bin/bash

conan install .. --build missing -pr arm-linux-gnueabihf-gcc-9 -s build_type=Release
cmake .. -DPLATFORM=linux-arm -DAPP=middleware -DCMAKE_BUILD_TYPE=Release "${@}"