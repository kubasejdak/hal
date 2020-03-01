#!/bin/bash

cmake .. -DPLATFORM=linux-arm -DAPP=middleware -DCMAKE_BUILD_TYPE=Release "${@}"
