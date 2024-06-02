#!/bin/bash

rm CMakeCache.txt CMakeFiles cmake_install.cmake Makefile -rf

cmake .

make

./joinMap

