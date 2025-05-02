#***********************************************************************
#* CMake file for the FASER2 Tracking module
#* This file creates the shared library for the FASER2 Tracking module
#* and the pybind11 bindings for the FASER2 Tracking module
#***********************************************************************

# Set the minimum version of CMake that's required
cmake_minimum_required(VERSION 3.12)

# Set the project name and version.
project(Tracking VERSION 0.0.0
                 DESCRIPTION "Module containing the Acts implenetation of the FASER2 tracking"
                 LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 20) 
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}")

# Search for ROOT and add all targets.
find_package(ROOT REQUIRED COMPONENTS Geom)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Geant4 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development)
add_subdirectory(pybind11)

# Fetch Acts release tarball from GitHub
include(FetchContent)
FetchContent_Declare(
    Acts
    URL https://github.com/acts-project/acts/archive/refs/tags/v39.2.0.tar.gz
    # URL_HASH MD5=f543dd8ba030bea2e4ee2f1b07dbe7c0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

# Set the Acts build options
set(ACTS_BUILD_PLUGIN_GEANT4 ON)
set(ACTS_BUILD_PLUGIN_ROOT ON)
set(ACTS_BUILD_PLUGIN_TGEO ON)
set(ACTS_BUILD_PLUGIN_ACTSVG ON)
# set(ACTS_BUILD_FATRAS_GEANT4 ON)
# set(ACTS_BUILD_FATRAS ON)
# set(ACTS_BUILD_FATRAS_ROOT ON)
set(ACTS_BUILD_EXAMPLES_PYTHON_BINDINGS ON)
set(ACTS_BUILD_EXAMPLES_GEANT4 ON)
FetchContent_MakeAvailable(Acts)
FetchContent_GetProperties(Acts)

# Set the include directories for the Acts compoment modules - you'll need to add to here when you want to include a new Acts module/plugin
include_directories(SYSTEM "${Acts_SOURCE_DIR}/Core/include")
include_directories(SYSTEM "${Acts_SOURCE_DIR}/Plugins/Geant4/include")

# Get the source, header and python binding files
file(GLOB SOURCE_FILES ${PROJECT_SOURCE_DIR}/source/*.cpp)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB PYTHON_FILES ${PROJECT_SOURCE_DIR}/python/*.py)
file(COPY share DESTINATION "${CMAKE_BINARY_DIR}")
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCE_FILES} ${HEADER_FILES} ROOT::GEOM ROOT::Core)

# Make the pybind11 module
pybind11_add_module(Tracking 
	${SOURCE_FILES}
	${HEADER_FILES}
)

# Compile the pybind11 module against the Acts, ROOT and Geant4 libraries
target_include_directories(Tracking PUBLIC ${ROOT_INCLUDE_DIRS} ${Geant4_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})
target_compile_definitions(Tracking PUBLIC ${Geant4_DEFINITIONS})
target_link_libraries(Tracking PUBLIC ActsCore ActsPluginGeant4 ${ROOT_LIBRARIES} ${Geant4_LIBRARIES})

# Copy scripts over to the build directory
set(Tracking_scripts
    setup.sh ${PYTHON_FILES}
  )

foreach(_script ${Tracking_scripts})
get_filename_component(_script_basename ${_script} NAME)
  configure_file(
    ${_script}
    ${PROJECT_BINARY_DIR}/${_script_basename}
    COPYONLY
    )
endforeach()
