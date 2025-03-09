#!/bin/bash
cd ~/VSCodeProjects/RadarControl
sudo rm -r build
rm -r proto/generated
mkdir build
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
make
