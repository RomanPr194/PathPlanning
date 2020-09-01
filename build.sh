#!/bin/sh

cd build
cmake .. -G "CodeLite - Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
make
./path_planning
