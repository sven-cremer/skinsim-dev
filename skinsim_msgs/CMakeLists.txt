cmake_minimum_required(VERSION 2.8.3)
project(skinsim_msgs)

set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

find_package(Protobuf REQUIRED)

set (msgs msgs/tactileData.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(skinsim_msgs SHARED ${PROTO_SRCS})
target_link_libraries(skinsim_msgs ${PROTOBUF_LIBRARY})
