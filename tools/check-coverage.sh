#!/bin/bash

COVERAGE_IGNORE="/usr/* */.conan/* /Library/Developer/CommandLineTools/*"

mkdir -p coverage
lcov -c -d . -o coverage/coverage.info
lcov -r coverage/coverage.info "${COVERAGE_IGNORE}" -o coverage/coverage.info

exit 0
