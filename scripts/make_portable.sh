#!/bin/bash
APP_PATH="/home/k1ps/Desktop/RadarControlApp"

mkdir -p $APP_PATH
cd $APP_PATH

cp -u -r /home/k1ps/VSCodeProjects/RadarControl/params .

mkdir -p flat
cp -u /home/k1ps/VSCodeProjects/RadarControl/flat/params.fbs flat

mkdir -p bin
cp -u /home/k1ps/VSCodeProjects/RadarControl/build/tools/bin/Tools bin
