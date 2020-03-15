# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (testtimer)
set(CMAKE_CXX_STANDARD 17)


add_executable (yourTarget test.c)


# Find OSQP library and headers
find_package(osqp REQUIRED)

# Link the OSQP shared library
target_link_libraries(yourTarget PRIVATE osqp::osqp)

