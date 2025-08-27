#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <grpcpp/grpcpp.h>
#include "calculator.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;
using calculator::Calculator;
using calculator::AddRequest;
using calculator::AddResponse;
using calculator::NumberRequest;
using calculator::NumberResponse;

// 客户端类
class CalculatorClient {
 public:
  CalculatorClient(std::shared_ptr<Channel> channel)
      : stub_(Calculator::NewStub(channel)) {}

  // 调用简单加法
  int Add(int a, int b) {
    AddRequest request;
    request.set_a(a);
    request.set_b(b);
    
    AddResponse response;
    ClientContext context;

    Status status = stub_->Add(&context, request, &response);

    if (status.ok()) {
      return response.result();
    } else {
      std::cout << "加法调用失败: " << status.error_code() << ": " << status.error_message()
                << std::endl;
      return -1;
    }
  }

  // 调用服务器流式接口：获取约数
  void GetDivisors(int num) {
    NumberRequest request;
    request.set_value(num);
    ClientContext context;

    std::unique_ptr<ClientReader<NumberResponse>> reader(
        stub_->GetDivisors(&context, request));

    NumberResponse response;
    std::cout << "获取 " << num << " 的约数: ";
    while (reader->Read(&response)) {
      std::cout << response.result() << " ";
    }
    std::cout << std::endl;

    Status status = reader->Finish();
    if (!status.ok()) {
      std::cout << "约数调用失败: " << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
  }

  // 调用客户端流式接口：累加
  int SumAll(const std::vector<int>& numbers) {
    ClientContext context;
    NumberResponse response;

    std::unique_ptr<ClientWriter<NumberRequest>> writer(
        stub_->SumAll(&context, &response));

    for (int num : numbers) {
      NumberRequest request;
      request.set_value(num);
      writer->Write(request);
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟数据发送间隔
    }
    writer->WritesDone();

    Status status = writer->Finish();
    if (status.ok()) {
      return response.result();
    } else {
      std::cout << "累加调用失败: " << status.error_code() << ": " << status.error_message()
                << std::endl;
      return -1;
    }
  }

  // 调用双向流式接口：计算平方
  void SquareNumbers(const std::vector<int>& numbers) {
    ClientContext context;

    std::shared_ptr<ClientReaderWriter<NumberRequest, NumberResponse>> stream(
        stub_->SquareNumbers(&context));

    // 启动一个线程发送数据
    std::thread writer_thread([stream, numbers]() {
      for (int num : numbers) {
        NumberRequest request;
        request.set_value(num);
        stream->Write(request);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }
      stream->WritesDone();
    });

    // 主线程接收响应
    NumberResponse response;
    std::cout << "平方计算结果: ";
    while (stream->Read(&response)) {
      std::cout << response.result() << " " << std::endl;
    }
    std::cout << std::endl;

    writer_thread.join();
    Status status = stream->Finish();
    if (!status.ok()) {
      std::cout << "平方调用失败: " << status.error_code() << ": " << status.error_message()
                << std::endl;
    }
  }

 private:
  std::unique_ptr<Calculator::Stub> stub_;
};

int main(int argc, char**argv) {
  CalculatorClient client(grpc::CreateChannel(
      "localhost:50051", grpc::InsecureChannelCredentials()));

  // 测试简单加法
  int add_result = client.Add(10, 20);
  std::cout << "10 + 20 = " << add_result << std::endl;

  // 测试服务器流式：获取约数
  client.GetDivisors(24);

  // 测试客户端流式：累加
  std::vector<int> numbers = {1, 2, 3, 4, 5};
  int sum_result = client.SumAll(numbers);
  std::cout << "1+2+3+4+5 = " << sum_result << std::endl;

  // 测试双向流式：计算平方
  std::vector<int> square_numbers = {1, 2, 3, 4, 5};
  client.SquareNumbers(square_numbers);

  return 0;
}
