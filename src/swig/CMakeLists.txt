# Add subdirectories for each language if desired
option(BUILD_PYTHON "Build Python SWIG module")
if(BUILD_PYTHON)
    find_package(SWIG REQUIRED)
    include(${SWIG_USE_FILE})
    add_subdirectory(python)
endif()