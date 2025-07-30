#include <iostream>
#include <map>
#include <string>

// 自定义比较器（可选）
struct ReverseCompare {
    bool operator()(const int& a, const int& b) const {
        return a > b; // 降序排列
    }
};

int main() {
    // 1. 创建一个 map，key 是 int，value 是 std::string，默认按 key 升序排序
    std::map<int, std::string> myMap;

    // 2. 插入元素（insert 和 operator[]）
    myMap[1] = "Apple";
    myMap[3] = "Banana";
    myMap.insert({2, "Cherry"});
    myMap.insert(std::make_pair(4, "Date"));

    // 3. 遍历 map（范围 for 循环）
    std::cout << "Contents of map:\n";
    for (const auto& pair : myMap) {
        std::cout << pair.first << " => " << pair.second << "\n";
    }

    // 4. 查找元素（find）
    int keyToFind = 3;
    auto it = myMap.find(keyToFind);
    if (it != myMap.end()) {
        std::cout << "Found key " << keyToFind << ": " << it->second << "\n";
    } else {
        std::cout << "Key " << keyToFind << " not found.\n";
    }

    // 5. 删除元素（erase）
    myMap.erase(2); // 删除 key 为 2 的元素
    std::cout << "After erasing key 2:\n";
    for (const auto& [key, val] : myMap) {
        std::cout << key << " => " << val << "\n";
    }

    // 6. 获取大小（size）和是否为空（empty）
    std::cout << "Map size: " << myMap.size() << "\n";
    std::cout << "Map is " << (myMap.empty() ? "empty" : "not empty") << "\n";

    // 7. 清空 map（clear）
    myMap.clear();
    std::cout << "Map cleared. Size: " << myMap.size() << "\n";

    // 8. 使用自定义比较器（降序）
    std::map<int, std::string, ReverseCompare> reversedMap;
    reversedMap[1] = "One";
    reversedMap[2] = "Two";
    reversedMap[3] = "Three";
    std::cout << "Reversed map:\n";
    for (const auto& [key, val] : reversedMap) {
        std::cout << key << " => " << val << "\n";
    }

    // 9. 使用 equal_range 获取范围
    auto range = reversedMap.equal_range(2);
    std::cout << "equal_range for key 2:\n";
    if (range.first != reversedMap.end())
        std::cout << "Lower bound: " << range.first->first << "\n";
    if (range.second != reversedMap.end())
        std::cout << "Upper bound: " << range.second->first << "\n";

    return 0;
}
