cmake_minimum_required(VERSION 3.16)

project(ugv-rover LANGUAGES CXX)

# Use C++17 to match existing compile script
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Output all executables into bin/
# Place executables in the CMake build directory (e.g., ./build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Dependencies inferred from existing compile script
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Try to find JsonCpp via pkg-config first (common on Linux)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(JSONCPP jsoncpp)
endif()

# Fallback to CMake config package name variants if available
if(NOT JSONCPP_FOUND)
  find_package(jsoncpp QUIET)
  if(TARGET jsoncpp_static OR TARGET jsoncpp_lib OR TARGET jsoncpp)
    set(JSONCPP_FOUND TRUE)
  endif()
endif()

include(CheckCXXSourceCompiles)

# Helper to attach jsoncpp to a target if available
function(link_jsoncpp target)
  if(JSONCPP_FOUND)
    if(TARGET jsoncpp_static)
      target_link_libraries(${target} PRIVATE jsoncpp_static)
    elseif(TARGET jsoncpp_lib)
      target_link_libraries(${target} PRIVATE jsoncpp_lib)
    elseif(TARGET jsoncpp)
      target_link_libraries(${target} PRIVATE jsoncpp)
    else()
      # pkg-config variables
      if(JSONCPP_LIBRARIES)
        target_link_libraries(${target} PRIVATE ${JSONCPP_LIBRARIES})
      endif()
      if(JSONCPP_INCLUDE_DIRS)
        target_include_directories(${target} PRIVATE ${JSONCPP_INCLUDE_DIRS})
      endif()
    endif()
  else()
    message(WARNING "JsonCpp not found. Targets depending on it may fail to link.")
  endif()
endfunction()

# Common libraries
set(UGV_COMMON_LIBS
  Boost::system
  Boost::thread
  Threads::Threads
)

# Helper macro to create an executable if the source exists
function(add_ugv_program name src)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/${src})
    add_executable(${name} src/${src})
    target_link_libraries(${name} PRIVATE ${UGV_COMMON_LIBS})
    # Link JsonCpp for targets that use on-device UI/JSON commands
    link_jsoncpp(${name})
  endif()
endfunction()

add_ugv_program(rover_control          rover_control.cpp)

target_include_directories(rover_control PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_sources(rover_control PRIVATE
  src/ui/OledViewModel.cpp
  src/input/TerminalInput.cpp
  src/sensors/LidarManager.cpp
  src/config/Config.cpp
)


