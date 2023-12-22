//
// Created by zyn on 2021/11/18.
//
#include <iostream>
#include <vector>
#include <math.h>
#include <set>
using namespace std;
struct DataPop{
    int data0 = 1;
    int data1 = 2;
};
int main()
{
    /// set usage test
    set<int> setTest;
    setTest.insert(3);
    setTest.insert(1);
    setTest.insert(0);
    setTest.insert(2);
    setTest.insert(3);
    cout << "last element end(): " << *setTest.end() << endl;
    cout << "last element begin(): " << *setTest.begin() << endl;
    cout << "last element: " << *setTest.rbegin() << endl;
    ///
    uint64_t value = 1403636579763559996;
    double valueDouble = value/1e9;
    cout.setf(ios::fixed);
    cout.precision(10);
    uint64_t valueInt = floor(valueDouble*1e5);
    cout << "valueDouble: " << valueDouble << endl;
    cout << "valueInt: " << valueInt << endl;
    cout << "value   : " << value<< endl;
    return 1;
    vector<DataPop*> test;
    DataPop d;
    DataPop *a ;
    DataPop *b = &d;
    test.push_back(a);
    test.push_back(b);
    test.pop_back();
    DataPop *c = test[1];
    cout << "size: " << test.size() << ",c " << c << endl;
    cout << "b: " << b << endl;
    if(c->data0)
    {
        cout << "value:" << c->data0 << endl;
    }
    return 1;
}