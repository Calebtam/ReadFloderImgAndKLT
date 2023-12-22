//
// Created by zyn on 2021/11/2.
//

#ifndef TEST_CLASSTEST1_H
#define TEST_CLASSTEST1_H

#include <functional>
#include <zconf.h>
#include <iostream>

struct Data{
    int age = 10;
};

typedef void (*obslam_result_callback)(Data& result);

typedef void (*resultCallback)(const Data* result);

class ClassTest1 {
public:
    void setAge()
    {
        Data data;
        data.age = 9;
        m_callBackFun(data);
        m_callback(data);
    }
    void setCallback(obslam_result_callback callback)
    {
        m_callback = callback;
    }
    void setCallbackPointer(resultCallback callback)
    {
        m_callbackPointer = callback;
    }
    void setAgePointer()
    {
        Data data;
        data.age = 9;
        m_callbackPointer(&data);

//        while(true)
//        {
//            sleep(2);
//            std::cout << "after callback !!" << std::endl;
//        }
    }

public:
    std::function<void(Data &)> m_callBackFun;
    obslam_result_callback m_callback;
    resultCallback m_callbackPointer;
};


#endif //TEST_CLASSTEST1_H
