#!/bin/bash

# Copyright (c) Facebook, Inc. and its affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

GIT_ROOT=$(git rev-parse --show-toplevel)
LIBFRANKA_VER=$1
LIBFRANKA_PATH="$GIT_ROOT/polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka"

# Check to make sure directory exists
[ ! -d $LIBFRANKA_PATH ] && echo "Directory $LIBFRANKA_PATH does not exist" && exit 1

# add
git -C "$GIT_ROOT" config url."https://github.com/".insteadOf git@github.com:
git -C "$GIT_ROOT" config url."https://github.com/".insteadOf ssh://git@github.com/
git -C "$GIT_ROOT" submodule sync --recursive
# ——add over ——

# Update libfranka version & submodules
cd $LIBFRANKA_PATH
if [ ! -z "$LIBFRANKA_VER" ]; then git checkout $LIBFRANKA_VER; fi
git submodule update --init --recursive
cd -

# Build
BUILD_PATH="${LIBFRANKA_PATH}/build"
if [ -d "$BUILD_PATH" ]; then rm -r $BUILD_PATH; fi
mkdir -p $BUILD_PATH && cd $BUILD_PATH
echo "Building libfranka at $BUILD_PATH"

cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ..
cmake --build .

cd -