rmdir build /Q /S
mkdir build
cd build
cmake -G "MinGW Makefiles" -DCMAKE_NOT_RPI_TARGET=1 ..
mingw32-make