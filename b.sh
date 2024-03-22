#!/bin/bash

if [ "$1" == "help" ]
  then
    echo "no argument - Start CrossForge"
    echo "d - Compile and run Debug version of CrossForge"
    echo "r - Compile and run Release version of CrossForge"
    echo "md - Compile Debug version of CrossForge"
    echo "mr - Compile Release version of CrossForge"
elif [ -z "$1" ] # check if an argument is given - "-z" checks if the length of the string is zero
  then
    echo "No argument supplied - CrossForge started in Release mode"
    cd out/build-lin
    ./CForgeSandbox
elif [ "$1" == "d" ]
  then
    echo "Compile and run Debug version of CrossForge"
    cd out/Debug
    make -j 12 && ./CForgeSandbox
elif [ "$1" == "r" ]
  then
    echo "Compile and run Release version of CrossForge"
    cd out/build-lin
    make -j 12 && ./CForgeSandbox
elif [ "$1" == "md" ]
  then
    echo "Compile Debug version of CrossForge"
    cd out/Debug
    make -j 12
elif [ "$1" == "mr" ]
  then
    echo "Compile Release version of CrossForge"
    cd out/build-lin
    make -j 12
elif [ "$1" == "cmake" ]
  then
    echo "Running CMake for Release"
    cmake -B out/build-lin -S . "-DCMAKE_TOOLCHAIN_FILE=/home/niclas/dev/vcpkg/scripts/buildsystems/vcpkg.cmake"
    echo "Running CMake for Debug"
    cmake -B out/Debug -S . "-DCMAKE_TOOLCHAIN_FILE=/home/niclas/dev/vcpkg/scripts/buildsystems/vcpkg.cmake" -DCMAKE_BUILD_TYPE=Debug
    echoe "CMake done"
else
  echo "Invalid argument - type 'help' for more information"
fi






