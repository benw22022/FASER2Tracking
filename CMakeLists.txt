
# Set the minimum version of CMake that's required
cmake_minimum_required(VERSION 3.12)

# Set the project name and version.
project(Tracking VERSION 0.0.0
                 DESCRIPTION "Module containing the sim code for the FASER2 trackers."
                 LANGUAGES CXX
)

set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")

# Search for ROOT and add all targets.
# set(EXPAT_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include")
# set(EXPAT_LIBRARY "$ENV{CONDA_PREFIX}/lib/libexpat.so")
# find_package(TBB REQUIRED)
find_package(ROOT REQUIRED COMPONENTS Geom)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Geant4 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development)
add_subdirectory(pybind11)

# if you want to try an alternate Acts, you can
# 1. comment out the stuff here and replace with add_subdirectory(acts)
# 2. git clone acts into this directory and choose your version
# find_package(Acts  REQUIRED)
# if (NOT Acts_FOUND)
  # message(STATUS "Did not find Acts, downloading and compiling it here")
include(FetchContent)
FetchContent_Declare(
    Acts
    URL https://github.com/acts-project/acts/archive/refs/tags/v41.0.0.tar.gz
    URL_HASH MD5=f543dd8ba030bea2e4ee2f1b07dbe7c0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(ACTS_BUILD_PLUGIN_GEANT4 ON)
set(ACTS_BUILD_PLUGIN_ROOT ON)
set(ACTS_BUILD_PLUGIN_ACTSVG ON)
set(ACTS_BUILD_FATRAS_GEANT4 ON)
set(ACTS_BUILD_FATRAS ON)
set(ACTS_BUILD_FATRAS_ROOT ON)
set(ACTS_BUILD_EXAMPLES_PYTHON_BINDINGS ON)
set(ACTS_BUILD_EXAMPLES_GEANT4 ON)
FetchContent_MakeAvailable(Acts)
FetchContent_GetProperties(Acts)
include_directories(SYSTEM "${Acts_SOURCE_DIR}/Core/include")
# else()
#   message(STATUS "Found Acts ${Acts_VERSION}")
# endif()

file(GLOB SOURCE_FILES ${PROJECT_SOURCE_DIR}/source/*.cpp)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB PYTHON_FILES ${PROJECT_SOURCE_DIR}/python/*.h, ${PROJECT_SOURCE_DIR}/python/*.cpp)
file(COPY share DESTINATION "${CMAKE_BINARY_DIR}")
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES} ${PYTHON_FILES} ROOT::GEOM ROOT::Core)

# include(pybind11.cmake)

pybind11_add_module(Tracking 
	${SOURCE_FILES}
	${HEADER_FILES}
	${PYTHON_FILES}
)

target_include_directories(Tracking PUBLIC ${ROOT_INCLUDE_DIRS} ${Geant4_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})
target_compile_definitions(Tracking PUBLIC ${Geant4_DEFINITIONS})
target_link_libraries(Tracking PUBLIC ActsCore ${ROOT_LIBRARIES} ${Geant4_LIBRARIES})

install(TARGETS Tracking DESTINATION python)
