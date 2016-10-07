#!/bin/bash
echo "Rebuilding project"
rm -rf build
mkdir build
cd build
cmake ..
make -j4
./main
