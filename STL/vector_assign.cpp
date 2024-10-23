#include <iostream>
#include <vector>

int main() {
    // 定义一个 float 数组
    float xyxy[6] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    
    // 预先定义好的 std::vector
    std::vector<float> vec(6, 0.0f);
    // 输出 vector 中的元素
    for (float value : vec) {
        std::cout << value << " ";
    }
    // vec.push_back(8.0f);
    // vec.push_back(9.0f);
    // std::cout << vec.size() << std::endl;

    // 使用 assign 函数将数组元素赋值给 vector
    vec.assign(xyxy, xyxy + 6);

    // 输出 vector 中的元素
    for (float value : vec) {
        std::cout << value << " ";
    }
    
    return 0;
}