#!/bin/bash

CLANG_VERSION=14

if [[ ! -f "compile_commands.json" ]]; then
    echo "No 'compile_commands.json' file found. Aborting."
    exit 1
fi

python3 ../tools/run-clang-tidy.py -clang-tidy-binary=clang-tidy-${CLANG_VERSION} \
                                   -clang-apply-replacements-binary=clang-apply-replacements-${CLANG_VERSION} \
                                   -quiet \
                                   -use-color \
                                   -header-filter=$(git rev-parse --show-toplevel)'(/app/|/demo/|/examples/|/lib/|/test/).*' \
                                   -export-fixes=errors.yml
