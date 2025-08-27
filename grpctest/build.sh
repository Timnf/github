#!/bin/bash

# 创建构建目录
mkdir -p build
cd build

# 运行CMake生成Makefile（会自动调用protoc生成代码）
cmake ..

# 编译项目
make

echo "编译完成，可执行文件在build目录下："
echo "运行服务器：./calculator_server"
echo "运行客户端：./calculator_client"
