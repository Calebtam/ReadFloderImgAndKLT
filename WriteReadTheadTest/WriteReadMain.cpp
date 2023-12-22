//
// Created by zyn on 2021/12/3.
//
#include <iostream>
#include <stdio.h>
#include <mutex>
#include <sys/prctl.h>
#include <thread>

using namespace std;
std::mutex data_mutex_;
int global_i =0;

void functionWrite()
{
    while(true)
    {
        {
            unique_lock<std::mutex> lock(data_mutex_);
            global_i++;
        }
        cout << "write_i: " << global_i <<endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds (1));
    }
}


void functionRead()
{
    while(true)
    {
        cout << "global_i: " << global_i <<endl;
    }
}

int main()
{

    thread thr1(functionWrite);
    thread thr2(functionRead);
    thr1.join();
    thr2.join();
    return 1;
}
