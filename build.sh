#!/bin/bash
set -e

rm -rf build
mkdir build
cd build 
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# check for argument
if [[ "$1" == "install" ]]; then
    echo "Running sudo make install..."
    sudo make install
fi

cd..