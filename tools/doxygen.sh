#!/bin/bash

while getopts n:v:i:o:m: flag; do
    case "${flag}" in
        n) NAME=${OPTARG};;
        v) VERSION=${OPTARG};;
        i) INPUT="${INPUT} ${OPTARG}";;
        o) OUTPUT=${OPTARG};;
        m) MAINPAGE=${OPTARG};;
    esac
done

if [ -z ${NAME+x} ]; then
    NAME=$(basename `git rev-parse --show-toplevel 2>/dev/null` 2>/dev/null)
    if [ $? != 0 ]; then
        echo "Missing NAME value (-n <NAME>). Aborting"
        exit 1
    fi
fi

if [ -z ${VERSION+x} ]; then
    VERSION=$(git describe --tags --dirty --broken --always 2>/dev/null)
    if [ $? != 0 ]; then
        echo "Missing VERSION value (-v <VERSION>). Aborting"
        exit 2
    fi
fi

if [ -z ${INPUT+x} ]; then
    echo "Missing INPUT value (-i <INPUT>). Aborting"
    exit 3
fi

if [ -z ${OUTPUT+x} ]; then
    OUTPUT=${PWD}/docs
fi

function update_config() {
    local KEY=$1
    local VALUE=$(echo $2 | sed 's#/#\\/#g')
    sed -i "s/${KEY} .*/${KEY} = ${VALUE}/" Doxyfile
}

doxygen -g

update_config "PROJECT_NAME" "${NAME}"
update_config "PROJECT_NUMBER" "${VERSION}"
update_config "OUTPUT_DIRECTORY" "${OUTPUT}"
update_config "INPUT" "${INPUT}"
update_config "EXTRACT_PRIVATE" "YES"
update_config "EXTRACT_PACKAGE" "YES"
update_config "EXTRACT_STATIC" "YES"
update_config "RECURSIVE" "YES"
update_config "EXCLUDE_PATTERNS" "*/.git/*  CMakeLists.txt"
if [ -n "${MAINPAGE}" ]; then
    update_config "USE_MDFILE_AS_MAINPAGE" "${MAINPAGE}"
fi
update_config "GENERATE_LATEX" "NO"
update_config "DOT_MULTI_TARGETS" "YES"
