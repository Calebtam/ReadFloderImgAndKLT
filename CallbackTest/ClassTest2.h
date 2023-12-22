//
// Created by zyn on 2021/11/2.
//

#ifndef TEST_CLASSTEST2_H
#define TEST_CLASSTEST2_H

#include "ClassTest1.h"

class ClassTest2 {
public:
    Data data;
    void getAge(Data &other)
    {
        data = other;
    }

};


#endif //TEST_CLASSTEST2_H
