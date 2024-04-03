@echo off
::REM
::REM
set "PYTHON_INCLUDE=C:\Python312\include"
set "PYTHON_LIB=C:\Python312\libs\python312.lib"
::REM
set "SWIG_DIR=<swig-dir>\swigwin-4.2.1"
set "SWIG_EXECUTABLE=<swig-dir>\swigwin-4.2.1\swig.exe"
::REM
SET PATH=%PATH%;%SWIG_DIR%
::REM call swig -help
pushd <path-to-this-repo>\solvespace_python\build
call cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_GUI=OFF -DENABLE_OPENMP=ON -DBUILD_PYTHON=ON
popd