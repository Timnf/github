#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <grpcpp/grpcpp.h>
#include "calculator.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using calculator::Calculator;
using calculator::AddRequest;
using calculator::AddResponse;
using calculator::NumberRequest;
using calculator::NumberResponse;

// 实现服务定义的接口
class CalculatorServiceImpl final : public Calculator::Service {
  // 简单加法实现
  Status Add(ServerContext* context, const AddRequest* request,
             AddResponse* response) override {
    response->set_result(request->a() + request->b());
    std::cout << "处理加法请求: " << request->a() << " + " << request->b() 
              << " = " << response->result() << std::endl;
    return Status::OK;
  }

  // 服务器流式：获取所有约数
  Status GetDivisors(ServerContext* context, const NumberRequest* request,
                     grpc::ServerWriter<NumberResponse>* writer) override {
    int num = request->value();
    std::cout << "处理约数请求: " << num << std::endl;
    
    for (int i = 1; i <= num; ++i) {
      if (num % i == 0) {
        NumberResponse response;
        response.set_result(i);
        writer->Write(response);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟处理时间
      }
    }
    return Status::OK;
  }

  // 客户端流式：累加所有数字
  Status SumAll(ServerContext* context, grpc::ServerReader<NumberRequest>* reader,
                NumberResponse* response) override {
    NumberRequest request;
    int sum = 0;
    std::cout << "开始处理累加请求" << std::endl;
    
    while (reader->Read(&request)) {
      sum += request.value();
      std::cout << "收到数字: " << request.value() << ", 当前总和: " << sum << std::endl;
    }
    
    response->set_result(sum);
    std::cout << "累加完成，结果: " << sum << std::endl;
    return Status::OK;
  }

  // 双向流式：计算平方
  Status SquareNumbers(ServerContext* context, 
                      grpc::ServerReaderWriter<NumberResponse, NumberRequest>* stream) override {
    NumberRequest request;
    std::cout << "开始处理平方请求" << std::endl;
    
    while (stream->Read(&request)) {
      int num = request.value();
      int square = num * num;
      NumberResponse response;
      response.set_result(square);
      stream->Write(response);
      std::cout << "收到: " << num << ", 计算平方: " << square << std::endl;
    }
    
    return Status::OK;
  }
};

// 启动服务器
void RunServer() {
  std::string server_address("0.0.0.0:50051");
  CalculatorServiceImpl service;

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "服务器启动，地址: " << server_address << std::endl;

  server->Wait();
}

int main(int argc, char**argv) {
  RunServer();
  return 0;
}
