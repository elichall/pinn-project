cmake_minimum_required(VERSION 3.10)
project(RoboticPINNSim VERSION 1.0 LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to Release mode if not specified to ensure -O3 optimizations
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Boost: required everywhere ---
find_package(Boost REQUIRED CONFIG)

# --- Eigen3: try system/nix, fall back to vendor/eigen/ ---
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  if(EXISTS "${CMAKE_SOURCE_DIR}/vendor/eigen/Eigen/Core")
    # Eigen is header-only; just point at the cloned repo
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/vendor/eigen"
    )
  elseif(EXISTS "${CMAKE_SOURCE_DIR}/vendor/eigen/CMakeLists.txt")
    add_subdirectory("${CMAKE_SOURCE_DIR}/vendor/eigen")
  else()
    message(FATAL_ERROR
      "Eigen3 not found.\n"
      "Nix users: add 'eigen' to buildInputs in your flake.\n"
      "Non-Nix users: clone https://gitlab.com/libeigen/eigen into vendor/eigen/")
  endif()
endif()

# --- InstanceSimEngine: try find_package, fall back to vendor/ ---
find_package(InstanceSimEngine QUIET)
if(NOT InstanceSimEngine_FOUND)
  if(EXISTS "${CMAKE_SOURCE_DIR}/vendor/instance-sim-engine/CMakeLists.txt")
    add_subdirectory("${CMAKE_SOURCE_DIR}/vendor/instance-sim-engine")
    # Create alias so users can use InstanceSimEngine::GraphicsEngine uniformly
    if(NOT TARGET InstanceSimEngine::GraphicsEngine)
      add_library(InstanceSimEngine::GraphicsEngine ALIAS GraphicsEngine)
    endif()
  else()
    message(FATAL_ERROR
      "InstanceSimEngine not found.\n"
      "Nix users: ensure the engine is in buildInputs in your flake.\n"
      "Non-Nix users: clone https://github.com/elichall/instance-sim-engine\n"
      "into vendor/instance-sim-engine/")
  endif()
endif()

# Suppress Eigen's use of deprecated std::result_of in C++17
add_compile_definitions(EIGEN_HAS_STD_RESULT_OF=0)

# Delegate the actual building of the code to the src_cpp folder.
add_subdirectory(src_cpp)
