//
// Created by zyn on 2023/12/13.
//

#include <iostream>
using namespace std;

// 基类 Animal
class Animal {
public:
    virtual void makeSound() {
      cout << "动物发出声音" << endl;
    }
    virtual void set() =0;
    int a = 0;
};

// 派生类 Dog
class Dog : public Animal {
public:
    Dog(){a = 10;}
    void set() { a = 2;}
    void makeSound() override {
      cout << "汪汪汪" << endl;
    }
    int b = 4;
};

// 派生类 Cat
class Cat : public Animal {
public:
    void set() { a = 1;}
    void makeSound() override {
      cout << "喵喵喵" << endl;
    }
};

int main() {
  Animal* animal1 = new Dog();
  Animal* animal2 = new Cat();

  animal1->makeSound(); // 输出：汪汪汪
  animal2->makeSound(); // 输出：喵喵喵
  std::cout << animal1->a << std::endl;
  animal1->set();
  std::cout << animal1->a << std::endl;
  std::cout << ((Dog*)animal1)->b << std::endl;

  delete animal1;
  delete animal2;

  return 0;
}
