cmake_minimum_required(VERSION 3.10)
project(calculator_grpc)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找gRPC和Protobuf库
find_package(PkgConfig REQUIRED)
find_package(Protobuf REQUIRED)
pkg_search_module(GRPC REQUIRED grpc)
pkg_search_module(GRPCPP REQUIRED grpc++)

# 查看grpc的路径和库
message(STATUS "GRPC_CPP_PLUGIN_EXECUTABLE: ${GRPC_CPP_PLUGIN_EXECUTABLE}")
message(STATUS "GRPC_LIBRARIES: ${GRPC_LIBRARIES}")
message(STATUS "GRPCPP_LIBRARIES: ${GRPCPP_LIBRARIES}")


# 找到grpc_cpp_plugin的路径
find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXECUTABLE)
  message(FATAL_ERROR "grpc_cpp_plugin not found")
endif()

# 生成proto文件对应的C++代码
get_filename_component(calc_proto "calculator.proto" ABSOLUTE)
get_filename_component(calc_proto_path "${calc_proto}" PATH)

# 生成的头文件和源文件路径
set(calc_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/calculator.pb.cc")
set(calc_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/calculator.pb.h")
set(calc_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/calculator.grpc.pb.cc")
set(calc_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/calculator.grpc.pb.h")

# 使用protobuf和grpc工具生成代码
add_custom_command(
  OUTPUT "${calc_proto_srcs}" "${calc_proto_hdrs}" "${calc_grpc_srcs}" "${calc_grpc_hdrs}"
  COMMAND protobuf::protoc
  ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
       --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
       -I "${calc_proto_path}"
       --plugin=protoc-gen-grpc="${GRPC_CPP_PLUGIN_EXECUTABLE}"
       "${calc_proto}"
  DEPENDS "${calc_proto}"
)

# 将生成的代码编译为静态库
add_library(calc_grpc_proto
  ${calc_proto_srcs}
  ${calc_proto_hdrs}
  ${calc_grpc_srcs}
  ${calc_grpc_hdrs}
)

# 链接依赖库
target_link_libraries(calc_grpc_proto
  ${GRPCPP_LIBRARIES}
  ${GRPC_LIBRARIES}
  protobuf::libprotobuf
)

# 包含必要的头文件目录
target_include_directories(calc_grpc_proto PUBLIC
  "${CMAKE_CURRENT_BINARY_DIR}"
  ${GRPC_INCLUDE_DIRS}
  ${GRPCPP_INCLUDE_DIRS}
)

# 编译服务器
add_executable(calculator_server server.cpp)
target_link_libraries(calculator_server
  calc_grpc_proto
  ${GRPCPP_LIBRARIES}
  ${GRPC_LIBRARIES}
  protobuf::libprotobuf
)

# 编译客户端
add_executable(calculator_client client.cpp)
target_link_libraries(calculator_client
  calc_grpc_proto
  ${GRPCPP_LIBRARIES}
  ${GRPC_LIBRARIES}
  protobuf::libprotobuf
)
