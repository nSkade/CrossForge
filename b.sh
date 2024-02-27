#!/bin/bash

if [ "$1" == "help" ]
  then
    echo "no argument - Start CrossForge"
    echo "d - Compile and run Debug version of CrossForge"
    echo "r - Compile and run Release version of CrossForge"
    echo "md - Compile Debug version of CrossForge"
    echo "mr - Compile Release version of CrossForge"
fi

# check if an argument is given - "-z" checks if the length of the string is zero
if [ -z "$1" ]
  then
    echo "No argument supplied - CrossForge started"
    cd out/build-lin
    ./CForgeSandbox
fi

if [ "$1" == "d" ]
  then
    echo "Compile and run Debug version of CrossForge"
    cd out/Debug
    make -j 12
    ./CForgeSandbox
fi

if [ "$1" == "r" ]
  then
    echo "Compile and run Release version of CrossForge"
    cd out/build-lin
    make -j 12
    ./CForgeSandbox
fi

if [ "$1" == "md" ]
  then
    echo "Compile Debug version of CrossForge"
    cd out/Debug
    make -j 12
fi

if [ "$1" == "mr" ]
  then
    echo "Compile Release version of CrossForge"
    cd out/build-lin
    make -j 12
fi





