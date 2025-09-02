#!/bin/bash
sudo rm -rf build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j ${nproc}
sudo make install
camera_publisher
