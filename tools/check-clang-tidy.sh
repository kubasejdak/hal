#!/bin/bash

if [[ ! -f "compile_commands.json" ]]; then
    echo "No 'compile_commands.json' file found. Aborting."
    exit 1
fi

python3 ../tools/run-clang-tidy.py -clang-tidy-binary=clang-tidy-11 -clang-apply-replacements-binary=clang-apply-replacements-11 -quiet -header-filter='hal/lib/.*' -export-fixes=errors.yml
