#!/bin/bash

conan install .. --build missing -pr clang-9 -s build_type=Release
cmake .. -DPLATFORM=linux -DAPP=hardware -DCMAKE_BUILD_TYPE=Release
