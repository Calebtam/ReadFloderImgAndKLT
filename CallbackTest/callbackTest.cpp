//
// Created by zyn on 2021/11/2.
//
#include "ClassTest2.h"

void getMsg(const Data *data2)
{
    std::cout << " getMsg:  "<< data2->age << std::endl;
    sleep(2);
    std::cout << " getMsg:  "<< data2->age << std::endl;
}

void getData(Data &data1)
{

    std::cout << " getData() "<< data1.age << std::endl;
    /// below do some process in callback function
}

int main()
{
    ClassTest1 test1;
    ClassTest2 test2;
    test1.m_callBackFun = std::bind(&ClassTest2::getAge, &test2,std::placeholders::_1);
    test1.setAge();
    std::cout << test2.data.age << std::endl;
    //
    test1.setCallback(getData);

    test1.setAge();

    test1.setCallbackPointer(getMsg);
    test1.setAgePointer();

    ///
    double timeStamp = 10e10;
    std::cout << "10e10: " << 10e10 << ", 1e10: " << 1e10 << ",e10: "<< "!! No e10 writing method !!" << std::endl;
    return 0;
}