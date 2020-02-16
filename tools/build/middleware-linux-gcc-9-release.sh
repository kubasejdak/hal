#!/bin/bash

conan install .. --build missing -pr gcc-9 -s build_type=Release
cmake .. -DPLATFORM=linux -DAPP=middleware -DCMAKE_BUILD_TYPE=Release "${@}"
