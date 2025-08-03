# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
# Get Git commit ID and version information
# First, try to get the most recent tag (for Semantic Versioning)
execute_process(
    COMMAND git describe --tags --abbrev=0
    OUTPUT_VARIABLE GIT_TAG_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE GIT_TAG_RESULT
)

# Get the full Git description (includes commits since tag and dirty state)
execute_process(
    COMMAND git describe --tags --dirty --always
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Process the version information
if(GIT_TAG_RESULT EQUAL 0)
    # Successfully found a tag - use it for the version number
    # Remove 'v' prefix if present (e.g., v1.2.3 -> 1.2.3)
    string(REGEX REPLACE "^v" "" VERSION_NUMBER "${GIT_TAG_VERSION}")
    message(STATUS "Using version from Git tag: ${VERSION_NUMBER}")
else()
    # No tag found - use a default version
    set(VERSION_NUMBER "0.9.24")
    message(STATUS "No Git tag found, using default version: ${VERSION_NUMBER}")
endif()

# Define macros for both Git commit ID and version number
add_compile_definitions(
    APP_VERSION="${GIT_VERSION}"
    APP_VERSION_NUMBER="${VERSION_NUMBER}"
)

message(STATUS "Building with version: ${VERSION_NUMBER} (${GIT_VERSION})")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS "D:/Projects/E32_Lora_Lib")
project(Rainsensor)





