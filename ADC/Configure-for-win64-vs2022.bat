@ECHO OFF
set PATH=D:/SystemVue/Tools/CMake/bin;%PATH%
@ECHO ON
mkdir build-win64-vs2022
cd build-win64-vs2022
cmake -DCMAKE_INSTALL_PREFIX=../output-vs2019 -G "Visual Studio 16" -A x64 ../source
cd ..
pause
