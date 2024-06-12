#include <iostream>
using namespace std;

// 将十进制数转换为指定进制的字符串表示
string convertToBase(int num, int base) {
    string result;
    do {
        int remainder = num % base;
        char digit = (remainder < 10) ? ('0' + remainder) : ('A' + remainder - 10);
        result = digit + result;
        num /= base;
    } while (num > 0);
    return result;
}

int main() {
    // 算术运算符
    int a = 10, b = 5;
    cout << "Arithmetic Operators:" << endl;
    cout << "a + b = " << a + b << endl;
    cout << "a - b = " << a - b << endl;
    cout << "a * b = " << a * b << endl;
    cout << "a / b = " << a / b << endl;
    cout << "a % b = " << a % b << endl;

    // 关系运算符
    cout << "\nRelational Operators:" << endl;
    cout << "a == b is " << (a == b) << endl;
    cout << "a != b is " << (a != b) << endl;
    cout << "a > b is " << (a > b) << endl;
    cout << "a < b is " << (a < b) << endl;
    cout << "a >= b is " << (a >= b) << endl;
    cout << "a <= b is " << (a <= b) << endl;

    // 逻辑运算符
    bool c = true, d = false;
    cout << "\nLogical Operators:" << endl;
    cout << "c && d is " << (c && d) << endl;
    cout << "c || d is " << (c || d) << endl;
    cout << "!c is " << !c << endl;
    cout << "!d is " << !d << endl;

    // 位运算符
    uint32_t x = 0b10000000100000001000000010000001;
    uint8_t y = 0b0000010;
    cout << "\nBitwise Operators:" << endl;
    cout << "x & y = " << convertToBase((x & y), 2) << endl;
    cout << "x | y = " << convertToBase((x | y), 2) << endl;
    cout << "x ^ y = " << convertToBase((x ^ y), 2) << endl;
    cout << "~x = " << convertToBase((~x), 2) << endl;
    cout << "x << 1 = " << convertToBase((x << 1), 2) << endl;
    cout << "y >> 1 = " << convertToBase((y >> 1), 2) << endl << endl;

    // uint32_t result = x; 
    // result |= static_cast<uint32_t>(vec[0]);
    // result |= static_cast<uint32_t>(vec[1]) << 8;
    // result |= static_cast<uint32_t>(vec[2]) << 16;
    // result |= static_cast<uint32_t>(vec[3]) << 24;


    cout << "x & y = " << (x & y) << endl;
    cout << "x | y = " << (x | y) << endl;
    cout << "x ^ y = " << (x ^ y) << endl;
    cout << "~x = " << (~x) << endl;
    cout << "x << 1 = " << (x << 1) << endl;
    cout << "y >> 1 = " << (y >> 1) << endl;
    return 0;
}
