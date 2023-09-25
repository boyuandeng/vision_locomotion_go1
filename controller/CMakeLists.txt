cmake_minimum_required(VERSION 2.8.3)
project(agile_locomotion)

add_compile_options(-std=c++17)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED COMPONENTS)
list(APPEND CMAKE_PREFIX_PATH "/home/ubuntu/anaconda3/envs/unitree/lib/python3.8/site-packages/torch/lib/")
find_package(Torch REQUIRED)
find_package(Boost COMPONENTS thread system filesystem REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )
find_package(Threads)
message(STATUS "Torch library status:${TORCH_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

catkin_package(
        LIBRARIES
        CATKIN_DEPENDS
)
catkin_simple(ALL_DEPS_REQUIRED)
include_directories("/home/ubuntu/anaconda3/envs/unitree/lib/python3.8/site-packages/torch/include")
link_directories("/home/ubuntu/anaconda3/envs/unitree/lib/python3.8/site-packages/torch/lib")

link_directories("lib")
# set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.a lcm)
cs_add_library(low_level src/low_level.cpp)
target_link_libraries(low_level 
                      "${catkin_LIBRARIES}"
                      "${EXTRA_LIBS}")

cs_add_library(cpp_policy src/cpp_policy.cpp)
target_link_libraries(cpp_policy
                      "${EXTRA_LIBS}"
                      "${Boost_LIBRARIES}"
                      "${catkin_LIBRARIES}"
                      "${CMAKE_THREAD_LIBS_INIT}"
                      "${TORCH_LIBRARIES}")
cs_add_executable(agile_locomotion src/agile_locomotion.cpp)
target_link_libraries(agile_locomotion 
                      low_level
                      cpp_policy
                      "${EXTRA_LIBS}"
                      "${TORCH_LIBRARIES}"
                      "${Boost_LIBRARIES}"
                      "${catkin_LIBRARIES}"
                      "${CMAKE_THREAD_LIBS_INIT}")


target_include_directories(agile_locomotion PUBLIC "${PROJECT_SOURCE_DIR}/include/agile_locomotion")


cs_install()
cs_export()

