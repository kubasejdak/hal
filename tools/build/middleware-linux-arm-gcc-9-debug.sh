#!/bin/bash

cmake .. -DPLATFORM=linux-arm -DAPP=middleware "${@}"
