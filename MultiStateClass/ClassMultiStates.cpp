//
// Created by zyn on 2022/6/8.
//
#include <iostream>
#include <string>
using namespace std;

class person
{
public:
    void print()    /// must exist to realize the multiple states
    {

    }
    int a;
};

class student: public person
{
public:
    string name;
    void print()
    {
        cout << "students !!" << endl;
    }
};

class work: public person
{
public:
    string workName;
    void print()
    {
        cout << "worker !!" << endl;
    }
};

void funtion1(person * people)
{
    people->print();
}


int main()
{

    return 0;
}