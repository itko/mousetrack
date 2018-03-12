# release
mkdir -p build && cd build && cmake .. && cd ..

# debug
mkdir -p build-debug && cd build-debug && cmake -DCMAKE_BUILD_TYPE=Debug .. && cd ..
