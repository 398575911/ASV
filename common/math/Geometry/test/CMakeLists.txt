# CMake 最低版本号要求
cmake_minimum_required (VERSION 3.10)

# 项目信息
project (Geomentry)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release") # "Debug" or "Release" mode
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -Weverything -Wextra -g -ggdb -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -march=native -mavx")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 添加 include 子目录

set(HEADER_DIRECTORY ${HEADER_DIRECTORY} 
   	"${PROJECT_SOURCE_DIR}/../include"
    )

set(LIBRARY_DIRECTORY ${LIBRARY_DIRECTORY} 
    "/usr/lib"
    "/usr/local/lib"
   )

# 指定生成目标


add_executable (miniball_example miniball_example.cc)

add_executable (miniball_example_containers miniball_example_containers.cc)


add_executable (testkdtree testkdtree.cc "${HEADER_DIRECTORY}/kdtree.c")

add_executable (testkdtree2 testkdtree2.cc "${HEADER_DIRECTORY}/kdtree.c")


add_executable (simple_voronoi simple_voronoi.cc)
target_include_directories(simple_voronoi PRIVATE ${HEADER_DIRECTORY})


set(RARE_LIBRARIES ${RARE_LIBRARIES} 
	"boost_system"
	"boost_filesystem"
	"boost_iostreams"
	"util"
	)
add_executable (plot_voronoi plot_voronoi.cc)
target_include_directories(plot_voronoi PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(plot_voronoi  ${RARE_LIBRARIES})


add_executable (plot_2dkdtree plot_2dkdtree.cc "${HEADER_DIRECTORY}/kdtree.c")
target_include_directories(plot_2dkdtree PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(plot_2dkdtree ${RARE_LIBRARIES})

add_executable (test_RScurve test_RScurve.cc)
target_include_directories(test_RScurve PRIVATE ${HEADER_DIRECTORY})
target_link_libraries(test_RScurve ${RARE_LIBRARIES})