#!/bin/bash
cd ~/VSCodeProjects/RadarControl
rm -r build
rm -r proto/generated
mkdir build
cd build
cmake ..
make
