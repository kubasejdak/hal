#!/bin/bash

conan install .. --build missing -pr arm-linux-gnueabihf-clang-9 -s build_type=Debug
cmake .. -DPLATFORM=linux-arm -DTOOLCHAIN=arm-linux-gnueabihf-clang-9 -DAPP=middleware "${@}"
