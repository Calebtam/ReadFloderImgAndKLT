//
// Created by zyn on 2022/7/15.
//

#include <iostream>
#include <vector>
#include <memory>

using namespace std;

struct PointerStrcut
{
    PointerStrcut(int _a, int *_p): a(_a), p(_p){}
    int a = 0;
    int *p;
};


struct frame
{
    int a;
    string b = "adj";
};

char* func1()
{
    char * ch1;
    ch1 = "abc";
    return ch1;
}

int* func2()
{
    int a = 100;
    int *b = &a;
    return b;
}

struct studient{
    string name  = "nihao !!";
    int age = 10 ;
};

int main()
{
    int a =100;
    int* p0 = &a;
    int* p1 = &a;
    int* p2 = p0;
    cout << "p0: " << p0 << ",p1: " << p1 << ",p2: " << p2 << endl;
    cout << "p0: " << *p0 << ",p1: " << *p1 << ",p2: " << *p2 << endl;
    ///

    PointerStrcut * p3 = new PointerStrcut(a,p0);
    PointerStrcut *p4 = p3;
    p3->p = nullptr;
    cout << p4->p<<endl;
    p3 = nullptr;
    ///
    int *lastFrame;
    {
        int current_num = 10;
        lastFrame = &current_num;
    }
    int aq[4] = {11,12,13,14};
    cout << "lastFrame: " << *lastFrame << endl;
    cout << "ch1: " << func1() <<",func2:" << *(func2()) <<endl;
    PointerStrcut last_p(a,p0);
    PointerStrcut curr_p(a,p0);
    curr_p.a = 100;
    cout << "address: " << &last_p <<",curr: " << &curr_p << endl;
    last_p = curr_p;
    cout << "address: " << &last_p <<",curr: " << &curr_p << endl;
    cout << "struct a: " << last_p.a << endl;
    ///
    std::shared_ptr<PointerStrcut> curr_ptr = std::make_shared<PointerStrcut>(a,p0);
    std::shared_ptr<PointerStrcut> last_ptr = curr_ptr;
    cout <<"last_ptr: " << last_ptr.get() << ",curr_ptra: " << curr_ptr.get() << endl;
    std::shared_ptr<PointerStrcut> curr_temp_ptr = std::make_shared<PointerStrcut>(a,p0);
    curr_temp_ptr->a = 1100;
    curr_ptr = curr_temp_ptr;
    cout <<"last_ptr->a: " << last_ptr->a << ",curr_ptr->a: " << curr_ptr->a << endl;
    cout <<"last_ptr: " << last_ptr.get() << ",curr_ptra: " << curr_ptr.get() << endl;
  {
    shared_ptr<studient> object = make_shared<studient>();
    studient zhangsan = *object;  /// deep copy, make sure
    object->name = "yes iam";
    cout << "name: " << zhangsan.name << endl;
    cout << "address1: " << &zhangsan << "," << object.get() << endl;
  }
    return 0;
}