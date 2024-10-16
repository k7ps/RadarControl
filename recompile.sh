#!/bin/bash
cd ~/VSCodeProjects/RadarControl
rm -r build
mkdir build
cd build
cmake ..
make
