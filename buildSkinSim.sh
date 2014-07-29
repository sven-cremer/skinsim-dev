#!/bin/bash

# Build skinsim_msgs
cd skinsim_msgs
mkdir build
cd build
cmake ../
make -j8

# Build skinsim_plugins
cd ../../skinsim_plugins
mkdir build
cd build
cmake ../
make -j8

# Build skinsim_test
cd ../../skinsim_test
mkdir build
cd build
cmake ../
make -j8

# Build skinsim_gen
cd ../../skinsim_gen
mkdir build
cd build
cmake ../
make -j8
