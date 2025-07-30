#include <iostream>
#include <vector>
#include <algorithm>  // for sort, find
#include <string>

struct Person {
    std::string name;
    int age;

    // 用于排序
    bool operator<(const Person& other) const {
        return age < other.age;
    }
};

int main() {
    // 1. 初始化和插入
    std::vector<int> numbers;
    numbers.push_back(10);
    numbers.push_back(20);
    numbers.push_back(30);
    numbers.emplace_back(40);  // 更高效的插入

    std::cout << "Initial vector:\n";
    for (int val : numbers)
        std::cout << val << " ";
    std::cout << "\n";

    // 2. 访问元素
    std::cout << "Element at index 2: " << numbers[2] << "\n";
    std::cout << "Element at index 1 (using at): " << numbers.at(1) << "\n";
    std::cout << "Front: " << numbers.front() << ", Back: " << numbers.back() << "\n";

    // 3. 插入到中间
    numbers.insert(numbers.begin() + 2, 25);
    std::cout << "After insert at index 2:\n";
    for (int val : numbers)
        std::cout << val << " ";
    std::cout << "\n";

    // 4. 删除元素
    numbers.erase(numbers.begin() + 1);  // 删除第 1 个元素
    numbers.pop_back();                  // 删除最后一个
    std::cout << "After deletion:\n";
    for (int val : numbers)
        std::cout << val << " ";
    std::cout << "\n";

    // 5. 遍历方式：迭代器
    std::cout << "Using iterators:\n";
    for (auto it = numbers.begin(); it != numbers.end(); ++it)
        std::cout << *it << " ";
    std::cout << "\n";

    // 6. 大小、容量
    std::cout << "Size: " << numbers.size() << ", Capacity: " << numbers.capacity() << "\n";

    // 7. resize, shrink_to_fit
    numbers.resize(6, -1); // 增加到6个元素，默认填充为 -1
    std::cout << "After resize to 6:\n";
    for (int val : numbers)
        std::cout << val << " ";
    std::cout << "\n";

    numbers.shrink_to_fit(); // 缩减容量以匹配当前大小

    // 8. 排序
    std::sort(numbers.begin(), numbers.end());
    std::cout << "After sort:\n";
    for (int val : numbers)
        std::cout << val << " ";
    std::cout << "\n";

    // 9. 查找元素
    auto it = std::find(numbers.begin(), numbers.end(), 25);
    if (it != numbers.end())
        std::cout << "Found 25 at index " << std::distance(numbers.begin(), it) << "\n";
    else
        std::cout << "25 not found.\n";

    // 10. 清空
    numbers.clear();
    std::cout << "After clear, size: " << numbers.size() << "\n";

    // 11. 使用自定义结构体的 vector
    std::vector<Person> people = {
        {"Alice", 25},
        {"Bob", 20},
        {"Charlie", 30}
    };

    std::sort(people.begin(), people.end()); // 按 age 排序

    std::cout << "Sorted people by age:\n";
    for (const auto& p : people)
        std::cout << p.name << " (" << p.age << ")\n";

    return 0;
}
