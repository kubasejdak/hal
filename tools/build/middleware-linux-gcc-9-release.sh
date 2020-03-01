#!/bin/bash

cmake .. -DPLATFORM=linux -DAPP=middleware -DCMAKE_BUILD_TYPE=Release "${@}"
