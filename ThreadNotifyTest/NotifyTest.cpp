//
// Created by zyn on 2021/10/18.
//

#include <iostream>
#include <stdio.h>
#include <mutex>
#include <sys/prctl.h>
#include <thread>
#include <condition_variable>
using namespace std;

std::mutex global_mutex;
std::condition_variable global_condition;
auto end_time = chrono::steady_clock::now();
auto start_time = chrono::steady_clock::now();
double accumulation_Time = 0.0;

int global_i = 0;

void function1()
{
    while(true)
    {
        double us = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count();
        accumulation_Time = accumulation_Time + us;
        if(global_i > 1000)
        {
            cout << "accumulation_Time: " << accumulation_Time << endl;
        }
        cout << "Cost Time: " << us << endl;

        std::unique_lock<std::mutex> lock(global_mutex);
        std::this_thread::sleep_for(std::chrono::milliseconds (1));
        cout << "Wait: " << global_i++ << endl;
        global_condition.wait(lock);
        end_time = chrono::steady_clock::now();
    }
}

void function2()
{
    while(true)
    {
        cout << "notify all: " << global_i << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
        std::unique_lock<std::mutex> lock(global_mutex);
        start_time = chrono::steady_clock::now();
        global_condition.notify_all();
        auto end_timeForNotityAll = chrono::steady_clock::now();
        double us = chrono::duration_cast<chrono::microseconds>(end_timeForNotityAll - start_time).count();
        cout << "notify all cost: " << us << endl;

    }
}

void function3()
{
    while(true)
    {
        cout << "notify one: " << global_i << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
        std::unique_lock<std::mutex> lock(global_mutex);
        start_time = chrono::steady_clock::now();
        global_condition.notify_one();
    }
}
#define NOTIFY_ALL_TEST
int main()
{
    /// mutex test
    std::mutex *mutexPtr1;
    std::mutex *mutexPtr2(mutexPtr1);
    return 1;
    thread t1(function1);
#ifdef NOTIFY_ALL_TEST
    thread t2(function2);  //  notify_all
#else
    thread t3(function3);  //  notify_one
#endif

    t1.join();
#ifdef NOTIFY_ALL_TEST
    t2.join();
#else
    t3.join();
#endif
    return 1;
}


/// result: notify one cost is same with  notify all