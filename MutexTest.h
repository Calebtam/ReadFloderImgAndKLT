//
// Created by zyn on 2021/9/8.
//

#ifndef TEST_MUTEXTEST_H
#define TEST_MUTEXTEST_H
#include <thread>
#include <iostream>
#include <mutex>
#include <zconf.h>
using namespace std;
namespace MutexTestSpace {

    class TestA
    {
    public:
        int b;
        int *a;
    };
    TestA testA, testB;
    mutex readClassData;
    void testClassVisitMethod()
    {
        int c = 4;
        testB.a = &c;
        testA.a = testB.a;
        testA.b = testB.b;
        unique_lock<mutex> lock(readClassData);
        while(true)
        {
            *testA.a = 5;
            std::this_thread::sleep_for(std::chrono::milliseconds (1000));
        }
    };

    void threadTestForClass()
    {
        thread th1(testClassVisitMethod);
        sleep(1);
        {
            unique_lock<mutex> lock(readClassData);
//            cout << "just run it " << endl;
            int as = 10;
//            testB.b = 10;
//            cout << "testB.b: " << testB.b << endl;
//            cout << "testB.a: " << testB.a << endl;
        }
        cout << "testB.b: " << testB.b << endl;
        th1.join();
    }

///  ****************  ///
    int b;
    int *a;
std::mutex readMutex;
void functionOutput()
{

    while(true) {
        {
            unique_lock<mutex> lock(readMutex);
            *a = 5;
            cout << a << endl;
            sleep(1);
        }
        sleep(1);
    }
}

void mutexTestForSimplePointer()
{
    b = 4;
    a = &b;
    thread th1(functionOutput);
    while(1){
        {
            unique_lock<mutex> lock(readMutex);
            *a = 6;
            cout << "b: " << *a << endl;
            sleep(1);
        }
        sleep(1);
    }
    cout << "test !!" << endl;
    th1.join();
}

};


#endif //TEST_MUTEXTEST_H
