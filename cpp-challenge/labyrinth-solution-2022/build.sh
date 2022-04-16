#! /bin/bash
echo "Removing Build folder."
rm -rf build

echo "Creating build folder."
mkdir build && cd build

echo -e "Generating make files.\n"
cmake .. 1> /dev/null

echo -e "Building ...\n"
make 1> /dev/null

echo -e "Running ...\n"
./gsoc_main
