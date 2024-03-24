# Include python

find_package(PythonInterp REQUIRED)
set(Python_ADDITIONAL_VERSIONS ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
message(STATUS "python version ${Python_ADDITIONAL_VERSIONS}")
IF (NOT DEFINED PYTHON_VERSION_STRING)
    find_package(PythonLibs REQUIRED)
ELSE (NOT DEFINED PYTHON_VERSION_STRING)
    find_package(PythonLibs ${PYTHON_VERSION_STRING} EXACT REQUIRED)
ENDIF(NOT DEFINED PYTHON_VERSION_STRING)

include_directories(${PYTHON_INCLUDE_PATH})

if (WIN32)
	set(PYLIB_SUFFIX .pyd)
else()
	set(PYLIB_SUFFIX .so)
endif()

set(PYTHON_PACKAGE "slvs")
set(LIBNAME _${PYTHON_PACKAGE}${PYLIB_SUFFIX})

set_source_files_properties(../slvs.i PROPERTIES CPLUSPLUS ON)
include_directories(../ ../../../include/)

# Add swig module
if (${CMAKE_VERSION} VERSION_LESS "3.8.0")
    swig_add_module(slvs python ../slvs.i)
else()
    swig_add_library(slvs LANGUAGE python SOURCES ../slvs.i)
endif()
swig_link_libraries(slvs slvs_static_excp ${PYTHON_LIBRARIES})

# set_target_properties(_slvs PROPERTIES
#     OUTPUT_NAME "_slvs"
#     SUFFIX ${PYLIB_SUFFIX})

# Files to install with Python
set(PYTHON_INSTALL_FILES slvs.py ${CMAKE_BINARY_DIR}/bin/${LIBNAME})

# Configure setup.py and copy to output directory
set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in)
set(SETUP_PY_OUT ${CMAKE_CURRENT_BINARY_DIR}/setup.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

# Install target to call setup.py
add_custom_target(install-python
        DEPENDS slvs
        COMMAND python ${SETUP_PY_OUT} install)