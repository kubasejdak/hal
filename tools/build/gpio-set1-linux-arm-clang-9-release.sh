#!/bin/bash

cmake .. -DPLATFORM=linux-arm -DTOOLCHAIN=arm-linux-gnueabihf-clang-9 -DAPP=gpio-set1 -DCMAKE_BUILD_TYPE=Release "${@}"
