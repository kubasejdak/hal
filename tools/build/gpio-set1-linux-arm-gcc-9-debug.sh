#!/bin/bash

cmake .. -DPLATFORM=linux-arm -DAPP=gpio-set1 "${@}"
