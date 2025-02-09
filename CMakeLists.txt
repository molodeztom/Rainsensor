# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
#Set version number from git
execute_process(
    COMMAND git describe --tags --dirty --always
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_compile_definitions(APP_VERSION="${GIT_VERSION}")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Rainsensor)





