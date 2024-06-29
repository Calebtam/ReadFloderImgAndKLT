#include <iostream>
#include <cmath>
#include <iostream>
#include <cmath>
#include <cassert>

class ParameterAdjuster {
public:
    // 构造函数
    ParameterAdjuster(float a, float b, float c) : a(a), b(b), c(c) {}

    // 获取参数的量级
    int getMagnitude(float value) {
        return (int) floor(log10(fabs(value)));
    }

    float adjust(float dst, float ref) {
        float max = ref / 3.0f;                         
        std::cout << "max: " << max << std::endl;
        float MagnitudeAdj = getMagnitude(dst);
        std::cout << "MagnitudeAdj: " << MagnitudeAdj << std::endl;
        std::cout << " getMagnitude(max): " <<  getMagnitude(max) << std::endl;
        float Factor = pow(10.0f, getMagnitude(max) - MagnitudeAdj);
        std::cout << "Factor: " << Factor << std::endl;
        float adjted = dst * Factor;
        std::cout << "adjted: " << adjted << std::endl;
        if (adjted >= max) {
            adjted = adjted / 10.0f;
        }
        return adjted;
    }

    // 运行测试
    void runTest() {
        // 计算A的当前量级
        int targetMagnitudeA = -2;
        int currentMagnitudeA = getMagnitude(a);

        // 计算调整系数
        float adjmentFactorA = pow(10.0f, targetMagnitudeA - currentMagnitudeA);
        // 调整A的值
        float adjedA = a * adjmentFactorA;
        float adjedB = adjust(b, adjedA);
        float adjedC = adjust(c, adjedA);

        std::cout << "Original A: " << a << std::endl;
        std::cout << "Original B: " << b << std::endl;
        std::cout << "Original C: " << c << std::endl;
        std::cout << "adjed A: " << adjedA << std::endl;
        std::cout << "adjed B: " << adjedB << std::endl;
        std::cout << "adjed C: " << adjedC << std::endl;

        // 验证调整后的结果
        assert(adjedB < adjedA * 4.0f);
        assert(adjedB < adjedA * 4.0f);
        std::cout << "All checks passed." << std::endl;
    }

private:
    float a;
    float b;
    float c;
};

int main() {
    // 测试用例
    ParameterAdjuster test1(0.048800f, 0.0120f, 0.044200f);
    // ParameterAdjuster test2(0.050567f, 0.01234f, 0.044200f);
    // ParameterAdjuster test3(1.234f, 0.00001234f, 4.5678f);

    std::cout << "Test Case 1" << std::endl;
    test1.runTest();
    std::cout << std::endl;

    // std::cout << "Test Case 2" << std::endl;
    // test2.runTest();
    // std::cout << std::endl;

    // std::cout << "Test Case 3" << std::endl;
    // test3.runTest();
    // std::cout << std::endl;

    return 0;
}

// public class Main {
//     public static void main(String[] args) {
//         // 测试用例
//         float[] testA = {12345.678f, 0.000567f, 1.234f};
//         float[] testB = {0.001234f, 1.234f, 0.00001234f};
//         float[] testC = {567.89f, 0.001234f, 4.5678f};

//         for (int i = 0; i < testA.length; i++) {
//             System.out.println("Test Case " + (i + 1));
//             runTest(testA[i], testB[i], testC[i]);
//             System.out.println();
//         }
//     }

//     // 运行测试
//     public static void runTest(float a, float b, float c) {
//         // 固定A的目标量级为1e-3
//         int targetMagnitudeA = -3;

//         // 计算A的当前量级
//         int currentMagnitudeA = getMagnitude(a);

//         // 计算调整系数
//         float adjustmentFactorA = (float) Math.pow(10, targetMagnitudeA - currentMagnitudeA);

//         // 调整A的值
//         float adjustedA = a * adjustmentFactorA;

//         // 调整B使其在[A/3, 3A]范围内
//         float adjustedB = adjustB(b, adjustedA);

//         // 调整C使其大于4A
//         float adjustedC = adjustC(c, adjustedA);

//         System.out.println("Original A: " + a);
//         System.out.println("Original B: " + b);
//         System.out.println("Original C: " + c);
//         System.out.println("Adjusted A: " + adjustedA);
//         System.out.println("Adjusted B: " + adjustedB);
//         System.out.println("Adjusted C: " + adjustedC);

//         // 验证调整后的结果
//         assert adjustedB >= adjustedA / 3 : "B is less than A/3";
//         assert adjustedB <= adjustedA * 3 : "B is greater than 3A";
//         assert adjustedC > adjustedA * 4 : "C is not greater than 4A";
//         System.out.println("All checks passed.");
//     }

//     // 获取 float 参数的量级
//     public static int getMagnitude(float value) {
//         return (int) Math.floor(Math.log10(Math.abs(value)));
//     }

//     // 调整B使其在[A/3, 3A]范围内
//     public static float adjustB(float b, float adjustedA) {
//         float minB = adjustedA / 3;
//         float maxB = adjustedA * 3;
//         float currentMagnitudeB = getMagnitude(b);
//         float adjustmentFactorB = (float) Math.pow(10, getMagnitude((minB + maxB) / 2) - currentMagnitudeB);
//         float adjustedB = b * adjustmentFactorB;
//         if (adjustedB < minB) {
//             adjustedB = minB;
//         } else if (adjustedB > maxB) {
//             adjustedB = maxB;
//         }
//         return adjustedB;
//     }

//     // 调整C使其大于4A
//     public static float adjustC(float c, float adjustedA) {
//         float minC = adjustedA * 4;
//         float currentMagnitudeC = getMagnitude(c);
//         float adjustmentFactorC = (float) Math.pow(10, getMagnitude(minC) - currentMagnitudeC);
//         float adjustedC = c * adjustmentFactorC;
//         if (adjustedC <= minC) {
//             adjustedC = minC;
//         }
//         return adjustedC;
//     }
// }
