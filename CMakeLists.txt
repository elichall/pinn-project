# 1. Set the minimum required version of CMake.
# 3.10 is a safe, stable baseline for modern Ubuntu/WSL systems.
cmake_minimum_required(VERSION 3.10)

# 2. Define the project name, its version, and the languages used.
# "CXX" tells CMake this is strictly a C++ project (no C or Fortran).
project(RoboticPINNSim VERSION 1.0 LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 3. Force the compiler to use the C++17 standard.
# This is crucial for your sequence lock (`std::atomic`) to work correctly.
set(CMAKE_CXX_STANDARD 17)

# 4. Throw a fatal error if the compiler doesn't support C++17.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 5. Disable compiler-specific extensions (like GNU's gnu++17).
# This ensures your code is strictly standard C++ and highly portable.
set(CMAKE_CXX_EXTENSIONS OFF)

# 6. Ask CMake to search your OS for the Boost library.
# The "REQUIRED" flag means CMake will crash immediately with a helpful error
# if it can't find Boost, rather than failing mysteriously later.
find_package(Boost REQUIRED CONFIG)

# 7. Delegate the actual building of the code to the src_cpp folder.
add_subdirectory(src_cpp)

# Suppress Eigen's use of deprecated std::result_of in C++17
add_compile_definitions(EIGEN_HAS_STD_RESULT_OF=0)
