//
// Created by zyn on 2023/6/12.
//
#include <iostream>
#include <vector>
#include <memory>

class Student{
public:
  Student(int b ){a = b;}
  int a = 10;
};

void function(void *p){
  std::shared_ptr<Student> student = *((std::shared_ptr<Student> *) p);
  std::cout << "use num: " << student.use_count() << std::endl;
  std::cout << "value: " << student->a << std::endl;
}

int main(){
  int* iPtr = new int(42);
  if(std::shared_ptr<int>(iPtr));
  int value = *iPtr; // Error! iPtr指针指向的内容已经被释放
  std::cout << value << std::endl;
  ///
  std::shared_ptr<Student> student = std::make_shared<Student>(20);
  function(&student);
  return 1;
}