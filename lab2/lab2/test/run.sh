#!/bin/bash
g++ test.cpp -o test -I/usr/include/pcl-1.10 -lpcl_io -lpcl_common -I/usr/include/eigen3
clear
./test

