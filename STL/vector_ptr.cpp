#include <iostream>
#include <memory>
#include <vector>
#include <deque>
#include <unordered_set>

class MybaseClass {
public:
    MybaseClass(int x) : x(x) {}
    void display() { std::cout << "MybaseClass value: " << x << std::endl; }
    int get_value() { return x; }
private:
    int x;
};

class MyClass : public MybaseClass{
public:
    MyClass(int x) : MybaseClass(x) {}
    // 修改构造函数，接受 shared_ptr 参数
    MyClass(std::shared_ptr<MybaseClass> ptr) : MybaseClass(ptr ? ptr->get_value() : 0) {
        // 如果传入 ptr 为空，使用默认值 0
    }
};

int main() {
    // 创建三个 shared_ptr
    std::shared_ptr<MybaseClass> ptr1 = std::make_shared<MybaseClass>(10);
    std::shared_ptr<MybaseClass> ptr2 = std::make_shared<MybaseClass>(20);
    std::shared_ptr<MybaseClass> ptr3 = std::make_shared<MybaseClass>(30);
    std::shared_ptr<MybaseClass> ptr4 = std::make_shared<MybaseClass>(40);
    std::shared_ptr<MybaseClass> ptr5 = std::make_shared<MybaseClass>(50);
    std::shared_ptr<MybaseClass> ptr6 = std::make_shared<MybaseClass>(60);

    std::cout << "Initial use_count(ptr1): " << ptr1.use_count() << std::endl; // 应该是 1
    std::cout << "Initial use_count(ptr2): " << ptr2.use_count() << std::endl; // 应该是 1
    std::cout << "Initial use_count(ptr3): " << ptr3.use_count() << std::endl; // 应该是 1

    // 创建一个 vector，存放 shared_ptr
    std::vector<std::shared_ptr<MybaseClass>> vec1, vec2;
    vec1.push_back(ptr1);  // 将 ptr1 添加到 vector 中
    vec1.push_back(ptr2);  // 将 ptr2 添加到 vector 中

    // 打印引用计数
    std::cout << "After push_back(ptr1), use_count(ptr1): " << ptr1.use_count() << std::endl; // 应该是 2
    std::cout << "After push_back(ptr2), use_count(ptr2): " << ptr2.use_count() << std::endl; // 应该是 2
    std::cout << "After push_back(ptr1), use_count(vec[0]): " << vec1[0].use_count() << std::endl; // 应该是 2
    std::cout << "After push_back(ptr2), use_count(vec[1]): " << vec1[1].use_count() << std::endl; // 应该是 2

    vec2.push_back(ptr1);  // 将 ptr1 添加到 vector 中
    vec2.push_back(ptr2);  // 将 ptr2 添加到 vector 中    
    // 打印引用计数
    std::cout << "After push_back(ptr1), use_count(ptr1): " << ptr1.use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr2), use_count(ptr2): " << ptr2.use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr1), use_count(vec[0]): " << vec1[0].use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr2), use_count(vec[1]): " << vec1[1].use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr1), use_count(vec[0]): " << vec2[0].use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr2), use_count(vec[1]): " << vec2[1].use_count() << std::endl; // 应该是 3


    // 现在添加 ptr3
    vec1.push_back(ptr3);  // 将 ptr3 添加到 vector 中
    // 打印引用计数
    std::cout << "After push_back(ptr3), use_count(ptr3): " << ptr3.use_count() << std::endl; // 应该是 2
    std::cout << "After push_back(ptr3), use_count(vec[2]): " << vec1[2].use_count() << std::endl; // 应该是 2

    vec2.push_back(vec1.back());  // 将 ptr3 添加到 vector 中
    // 打印引用计数
    std::cout << "After push_back(ptr3), use_count(ptr3): " << ptr3.use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr3), use_count(vec[2]): " << vec1[2].use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr3), use_count(vec[2]): " << vec2[2].use_count() << std::endl; // 应该是 3


    std::vector<std::shared_ptr<MyClass>> vec3, vec4;
    // 这里的 ptr1 被传递到 MyClass 的构造函数中，创建了一个新的 shared_ptr<MyClass>（即 vec3[0]），它指向一个 MyClass 对象。
    // 这个新创建的 shared_ptr (vec3[0]) 会持有对 MyClass 对象的所有权，但它并没有直接增加 ptr1 对应的 MybaseClass 对象的引用计数。
    // 这很重要，因为 MyClass 继承自 MybaseClass，所以 ptr1 持有的是 MybaseClass 类型的对象，而 vec3[0] 是一个指向 MyClass 类型的对象。
    
    // vec3.push_back(std::make_shared<MyClass>(ptr1)) 创建的 shared_ptr<MyClass>（存储在 vec3[0] 中）是一个 新的 shared_ptr<MyClass>，
    // 并且它是 ptr1 的副本。它指向一个新的 MyClass 对象，这个对象通过 ptr1 的值（ptr1 指向的 MybaseClass 对象）初始化，但是 vec3[0] 并没有持有 ptr1 指向的原始对象 MybaseClass，而是持有新的 MyClass 类型的对象。
    vec3.push_back(std::make_shared<MyClass>(ptr1));
    std::cout << "After push_back(ptr1), use_count(ptr1): " << ptr1.use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr1), use_count(vec3[0]): " << vec3[0].use_count() << std::endl; // 应该是 1
    std::cout << "After push_back(ptr1), get(ptr1): " << ptr1.get() << std::endl;
    std::cout << "After push_back(ptr1), get(vec3[0]): " << vec3[0].get() << std::endl;
    // After push_back(ptr1), get(ptr1): 0x55f99f866ec0
    // After push_back(ptr1), get(vec3[0]): 0x55f99f867390
    
    vec4.push_back(vec3.back());
    std::cout << "After push_back(ptr1), use_count(ptr1): " << ptr1.use_count() << std::endl; // 应该是 3
    std::cout << "After push_back(ptr1), use_count(vec4[0]): " << vec4[0].use_count() << std::endl; // 应该是 2


    // vec5 是一个存储父类指针的 std::vector，而 vec6 存储子类指针。
    // 由于父类和子类的对象地址不同（std::shared_ptr 管理的是实际对象的地址，而非它们的基类部分的地址），简单的直接比较指针会失败。    
    std::vector<std::shared_ptr<MybaseClass>> vec5;
    std::deque<std::shared_ptr<MyClass>> vec6;
    vec5.push_back(ptr1);  // 将 ptr1 添加到 vector 中
    vec5.push_back(ptr2);  // 将 ptr2 添加到 vector 中
    vec5.push_back(ptr3);  // 将 ptr3 添加到 vector 中
    // vec5.push_back(ptr4);  // 将 ptr4 添加到 vector 中
    // vec5.push_back(ptr5);  // 将 ptr5 添加到 vector 中
    // vec5.push_back(ptr6);  // 将 ptr6 添加到 vector 中

    vec6.push_back(std::make_shared<MyClass>(ptr2));
    vec6.push_back(std::make_shared<MyClass>(ptr4));
    vec6.push_back(std::make_shared<MyClass>(ptr5));
    vec6.push_back(std::make_shared<MyClass>(ptr6));

    // for (const auto& ptr : vec5) {
    //     if (ptr) ptr->display();
    // }

    // for (const auto& ptr : vec6) {
    //     if (ptr) ptr->display();
    // }

    // 将 vec5 的值存入哈希表
    std::unordered_set<int> base_values;
    for (const auto& base_ptr : vec5) {
        if (base_ptr) base_values.insert(base_ptr->get_value());
    }

    // 遍历 vec6 并查找共有元素
    for (const auto& child_ptr : vec6) {
        if (child_ptr && base_values.count(child_ptr->get_value())) {
            std::cout << "共有值: " << child_ptr->get_value() << std::endl;
        }
    }


    return 0;
}
