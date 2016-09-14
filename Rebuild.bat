rmdir build /Q /S
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make