#!/bin/bash
cd ~/VSCodeProjects/RadarControl
rm -r build
rm -r flat/generated
mkdir build
cd build
cmake ..
make
