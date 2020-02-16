#!/bin/bash

conan install .. --build missing -pr arm-linux-gnueabihf-clang-9 -s build_type=Release
cmake .. -DPLATFORM=linux-arm -DTOOLCHAIN=arm-linux-gnueabihf-clang-9 -DAPP=middleware -DCMAKE_BUILD_TYPE=Release "${@}"
