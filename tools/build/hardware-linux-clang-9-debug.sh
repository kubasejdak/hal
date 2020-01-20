#!/bin/bash

conan install .. --build missing -pr clang-9 -s build_type=Debug
cmake .. -DPLATFORM=linux -DAPP=hardware
