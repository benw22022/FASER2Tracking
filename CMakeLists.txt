
# Set the minimum version of CMake that's required
cmake_minimum_required(VERSION 3.12)

# Set the project name and version.
project(Tracking VERSION 0.0.0
                 DESCRIPTION "Module containing the sim code for the FASER2 trackers."
                 LANGUAGES CXX
)

# Search for ROOT and add all targets.
find_package(ROOT CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Geant4 REQUIRED)
find_package(ZLIB REQUIRED)
message(STATUS "ZLIB_LIBRARIES: ${ZLIB_LIBRARIES}")
message(STATUS "ZLIB_INCLUDE_DIRS: ${ZLIB_INCLUDE_DIRS}")

file(GLOB sources ${PROJECT_SOURCE_DIR}/source/*.cpp)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.h)


# if you want to try an alternate Acts, you can
# 1. comment out the stuff here and replace with add_subdirectory(acts)
# 2. git clone acts into this directory and choose your version
find_package(Acts QUIET)
if (NOT Acts_FOUND)
  message(STATUS "Did not find Acts, downloading and compiling it here")
  include(FetchContent)
  FetchContent_Declare(
      Acts
      URL https://github.com/acts-project/acts/archive/refs/tags/v36.0.0.tar.gz
      URL_HASH MD5=f543dd8ba030bea2e4ee2f1b07dbe7c0
  )
  FetchContent_MakeAvailable(Acts)
  FetchContent_GetProperties(Acts)
  # Adding Acts as "SYSTEM" 
  # which will silence compiler warnings from these 3rd party softwares
  include_directories(SYSTEM "${Acts_SOURCE_DIR}/Core/include")
else()
  message(STATUS "Found Acts ${Acts_VERSION}")
endif()


add_library(Tracking SHARED ${sources})
target_include_directories(Tracking PUBLIC ${include})
include_directories(${ROOT_INCLUDE_DIRS} ${Geant4_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})
target_compile_definitions(Tracking PUBLIC ${Geant4_DEFINITIONS})

target_link_libraries(Tracking PUBLIC ActsCore ${ROOT_LIBRARIES} ${Geant4_LIBRARIES} ${ZLIB_LIBRARIES})


# target_link_libraries(Tracking PUBLIC ${ZLIB_LIBRARIES})



# setup_python(package_name FASER2/Tracking)