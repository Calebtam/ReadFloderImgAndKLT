#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <algorithm>  // 为了使用 std::remove_if

int main() {
    // 定义 map
    std::map<uint64_t, std::vector<std::pair<uint64_t, std::string>>> myMap;

    // 1. 多层插入
    myMap[1].push_back({100, "Apple"});
    myMap[1].push_back({101, "Banana"});
    myMap[2].push_back({200, "Cherry"});
    myMap[2].push_back({201, "Date"});
    myMap[3].push_back({300, "Elderberry"});

    std::cout << "After insertion:\n";

    // 2. 遍历
    for (const auto& [key, vec] : myMap) {
        std::cout << "Key: " << key << "\n";
        for (const auto& [id, name] : vec) {
            std::cout << "  (" << id << ", " << name << ")\n";
        }
    }

    // 3. 访问元素（访问特定 key 下的 vector 元素）
    uint64_t outerKey = 2;
    uint64_t innerId = 201;
    std::cout << "\nAccessing item with outer key " << outerKey << " and inner id " << innerId << ":\n";
    if (myMap.find(outerKey) != myMap.end()) {
        for (const auto& [id, name] : myMap[outerKey]) {
            if (id == innerId) {
                std::cout << "Found inner item: " << name << "\n";
            }
        }
    } else {
        std::cout << "Outer key not found.\n";
    }

    // 4. 删除元素
    // 删除 outer key 3 的整个 vector
    // 在执行 myMap.erase(3); 之后，key == 3 的空间（键值对）已完全从 std::map 中移除，包括：
    //     外层的 key 3
    //     内层的 vector<pair<uint64_t, string>>
    // 这意味着：
    //     myMap.count(3) 会返回 0
    //     myMap.find(3) 会返回 myMap.end()
    myMap.erase(3);
    std::cout << "\nAfter erasing outer key 3:\n";

    // 删除 outer key 1 下 innerId 为 100 的元素
    if (myMap.find(1) != myMap.end()) {
        auto& vec = myMap[1];
        vec.erase(std::remove_if(vec.begin(), vec.end(),
                                 [](const std::pair<uint64_t, std::string>& p) {
                                     return p.first == 100;
                                 }),
                  vec.end());
    }

    std::cout << "After erasing inner id 100 under outer key 1:\n";

    // 5. 遍历查看删除后的 map
    for (const auto& [key, vec] : myMap) {
        std::cout << "Key: " << key << "\n";
        for (const auto& [id, name] : vec) {
            std::cout << "  (" << id << ", " << name << ")\n";
        }
    }

    // 6. 查询大小
    std::cout << "\nMap size (outer keys): " << myMap.size() << "\n";
    for (const auto& [key, vec] : myMap) {
        std::cout << "Outer key " << key << " has " << vec.size() << " inner elements.\n";
    }

    // 7. 清空 map
    myMap.clear();
    std::cout << "\nMap cleared. Size: " << myMap.size() << "\n";

    return 0;
}
