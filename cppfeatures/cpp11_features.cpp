#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>

// 1. 智能指针示例
void smart_pointer_example()
{
    std::cout << "Smart Pointer Example:" << std::endl;

    // 使用std::unique_ptr 管理动态分配的内存 独占所有权
    std::unique_ptr<int> uptr(new int(42));
    std::cout << "Unique Pointer: " << *uptr << std::endl;
    auto uptr2 = std::move(uptr);
    std::cout << "Unique Pointer after move: " << *uptr2 << std::endl;
    uptr = std::move(uptr2);
    // uptr3 = uptr2; // 编译错误 unique_pt
    std::cout << "Unique Pointer after move back: " << *uptr << std::endl;
    std::cout << std::endl;

    // 使用std::shared_ptr 管理动态分配的内存 共享所有权
    std::shared_ptr<int> sptr(new int(42));
    std::cout << "Shared Pointer: " << *sptr << std::endl;
    std::cout << "Shared Pointer Count: " << sptr.use_count() << std::endl;
    auto sptr2 = sptr;
    std::cout << "Shared Pointer Count: " << sptr.use_count() << std::endl;
    std::cout << std::endl;

    // 使用std::weak_ptr 管理共享指针的弱引用 不增加引用计数 解决循环引用问题
    std::weak_ptr<int> wptr = sptr;
    if (auto sp = wptr.lock())
    {
        std::cout << "Weak Pointer: " << *sp << std::endl;
    }
    else
    {
        std::cout << "Weak Pointer expired." << std::endl;
    }
    std::cout << "Shared Pointer Count after weak_ptr: " << sptr.use_count() << std::endl;
    sptr.reset();
    if (auto sp = wptr.lock())
    {
        std::cout << "Weak Pointer after shared_ptr reset: " << *sp << std::endl;
    }
    else
    {
        std::cout << "Weak Pointer expired after shared_ptr reset." << std::endl;
    }
    std::cout << "Shared Pointer Count after weak_ptr lock: " << sptr.use_count() << std::endl;
    std::cout << std::endl;
}

//2. 移动语义示例 move语义 避免不必要的拷贝
void move_semantics_example()
{
    std::cout << "Move Semantics Example:" << std::endl;
    std::vector<int> src = {1, 2, 3, 4, 5};
    std::vector<int> dst = std::move(src);
    std::cout << "Source vector size: " << src.size() << std::endl;
    std::cout << "Destination vector size: " << dst.size() << std::endl;
    std::cout << std::endl;
    std::cout << "Destination vector: ";
    for (auto &i : dst)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;


}


// 3. lambda表达式示例 匿名函数 可以捕获外部变量 可以作为参数传递 可以作为返回值 可以作为函数对象 
void lambda_example()
{
    std::cout << "Lambda Expression Example:" << std::endl;
    auto add = [](int a, int b)
    { return a + b; };

    std::cout << "Addition: " << add(3, 4) << std::endl;
    std::cout << std::endl;

    std::vector<int> numbers = {1, 2, 3, 4, 5};
    std::for_each(numbers.begin(), numbers.end(), [](int &n)
    { n *= 2; });
    std::cout << "Doubled numbers: ";

    for (auto &n : numbers)
    {
        std::cout << n << " ";
    }

    // 带捕获和返回值的lambda表达式
    std::function<int(int)> fibonacci = [&](int n) -> int
    {
        if (n <= 1)
            return n;
        return fibonacci(n - 1) + fibonacci(n - 2);
    };
    std::cout << "\nFibonacci(5): " << fibonacci(5) << std::endl;

    std::cout << std::endl;

}

// 4. 并发编程示例 std::thread std::mutex std::condition_variable std::future
void concurrent_programming_example()
{
    std::cout << "Concurrent Programming Example:" << std::endl;
    std::mutex mtx;
    std::condition_variable cv;
    bool ready = false;
    int shared_data = 0;
    std::cout << "Main thread is running..." << std::endl;


    std::thread t([&]()
    {
        std::unique_lock<std::mutex> lock(mtx);
        std::cout << "Thread is running..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
        ready = true;
        shared_data += 1;
        cv.notify_one();
        lock.unlock();
        std::cout << "Thread is done." << std::endl;
    });

        std::thread t2([&]()
    {
        std::unique_lock<std::mutex> lock(mtx);
        std::cout << "Thread2 is running..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        ready = true;
        shared_data += 1;
        cv.notify_one();
        lock.unlock();
        std::cout << "Thread2 is done." << std::endl;
    });

    std::cout << "Main thread is waiting..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&]()
    { return ready; });
    std::cout << "Main thread notified." << std::endl;
    std::cout << "Shared data: " << shared_data << std::endl;
    lock.unlock();
    t.join();
    t2.join();
    std::cout << std::endl;
}



int main()
{
    smart_pointer_example();
    move_semantics_example();
    lambda_example();
    concurrent_programming_example();

    return 0;
}