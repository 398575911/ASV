# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testGPS)
set(CMAKE_CXX_STANDARD 17)


# 添加 include 子目录
set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
	"${PROJECT_SOURCE_DIR}/../../../../")

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
   )
    
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../../../common/logging/src/easylogging++.cc")  

find_library(SERIALPORT_LIBRARY serialport HINTS ${LIBRARY_DIRECTORY})


# 指定生成目标
add_executable (power_test power_test.cc ${SOURCE_FILES})
target_include_directories(power_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(power_test PUBLIC ${SERIALPORT_LIBRARY})


