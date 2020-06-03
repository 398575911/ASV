# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.8)

# 项目信息
project (testcommunication)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Debug") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
   	"${PROJECT_SOURCE_DIR}/../../../"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib/"
    "/usr/local/lib/"
   )
 
set(SOURCE_FILES ${SOURCE_FILES} 
	"${PROJECT_SOURCE_DIR}/../../logging/src/easylogging++.cc" )

find_library(SERIALPORT_LIBRARY serialport HINTS ${LIBRARY_DIRECTORY})



# 指定生成目标
add_executable (crc_test CRC_test.cc)
target_include_directories(crc_test PRIVATE ${HEADER_DIRECTORY})


add_executable (simple_crc_test simple_crc_test.cc)
target_include_directories(simple_crc_test PRIVATE ${HEADER_DIRECTORY})


add_executable (testtcpserver testtcpserver.cc  ${SOURCE_FILES})
target_include_directories(testtcpserver PRIVATE ${HEADER_DIRECTORY})

add_executable (testtcpclient testtcpclient.cc)
target_include_directories(testtcpclient PRIVATE ${HEADER_DIRECTORY})


add_executable (serial_test serial_test.cc ${SOURCE_FILES} )
target_include_directories(serial_test PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(serial_test PRIVATE ${SERIALPORT_LIBRARY})

add_executable (dataserial_test dataserial_test.cc)
target_include_directories(dataserial_test PRIVATE ${HEADER_DIRECTORY})