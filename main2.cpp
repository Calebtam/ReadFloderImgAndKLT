#include <iostream>
#include <stdio.h>
#include <mutex>
#include <sys/prctl.h>
#include <thread>


using namespace std;
/// three kinds of methods to solve the const impact
class A{
public:
    int GetData() const
    {
        std::unique_lock<std::mutex> lock(*const_cast<std::mutex *>( &data_mutex_) );
        return a_;
    }
    static void foo();
private:
    const int a_ = 1;
    std::mutex data_mutex_;
//    static std::mutex data_mutex_;
    /// or
//    mutable std::mutex data_mutex_;
};
/// not use the static key words, you can also assign value in class
//std::mutex A::data_mutex_;

/// note: do not use the static key word
void A::foo(){}


void threadName()
{
#include <sys/prctl.h>
    const char *new_name = "NameThread";
    prctl(PR_SET_NAME, new_name);

    while(true)
    {
        std::cout << "in recycle !!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (1));
    }
}


#include "MutexTest.h"

int main(){
//    MutexTestSpace::threadTestForClass();
    MutexTestSpace::mutexTestForSimplePointer();
    exit(1);
    thread th1(threadName);

    A a;
    cout << a.GetData();
    while(true)
    {
        std::cout << "main thread in recycle !!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (1));
    }
    th1.join();
    return 0;
}

