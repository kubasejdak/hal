#!/bin/bash

cmake .. -DPLATFORM=linux-arm -DAPP=gpio-set1 -DCMAKE_BUILD_TYPE=Release "${@}"
