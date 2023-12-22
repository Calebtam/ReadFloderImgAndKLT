//
// Created by zyn on 2022/7/13.
//

#include <iostream>
#include <stdio.h>
#include <mutex>
#include <unistd.h>
#include <sys/prctl.h>
#include <thread>
#include <condition_variable>
using namespace std;

std::mutex global_mutex;

struct Person{
    string name;
    int age;
};

Person personObj = {"haha",14};
void function11()
{

    unique_lock<mutex> lock(global_mutex);
    personObj.name = "zyn";

    while(true)
    {
        sleep(1);
        cout << "name: " << personObj.name << endl;
    }
}
//string &name = personObj.name;
void function22()
{
    string name;
    {
        unique_lock<mutex> lock(global_mutex);
        name = "god";
    }
    cout << "name: " << name << endl;
    sleep(1);
}

// int main()
// {
//     thread thr1(function11);
//     sleep(1);
//     cout << "name: " << personObj.name << endl;
//     thread thr2(function22);
//     sleep(1000);
//     thr2.join();
//     thr1.join();
// //    cout << "after thread name: " << name << endl;
//     return 0;
// }