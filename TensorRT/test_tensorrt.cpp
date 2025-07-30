#include <iostream>
#include <fstream>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <vector>
// Error handling macro
#define CHECK(status) \
    if (status != 0) \
    { \
        std::cerr << "Cuda failure: " << status << std::endl; \
        abort(); \
    }

class Logger : public nvinfer1::ILogger {
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override {
        if (severity != nvinfer1::ILogger::Severity::kINFO) {
            std::cout << msg << std::endl;
        }
    }
};

// 加载 TensorRT 引擎
nvinfer1::ICudaEngine* loadEngine(const std::string& engineFile) {
    std::ifstream file(engineFile, std::ios::binary);
    if (!file) {
        std::cerr << "Error opening engine file: " << engineFile << std::endl;
        return nullptr;
    }

    // 获取文件大小
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    // 读取引擎数据
    std::vector<char> engineData(size);
    file.read(engineData.data(), size);

    // 创建 TensorRT runtime
    Logger logger;
    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(logger);

    if (!runtime) {
        std::cerr << "Failed to create runtime" << std::endl;
        return nullptr;
    }

    // 反序列化引擎
    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engineData.data(), size, nullptr);
    if (!engine) {
        std::cerr << "Failed to deserialize engine" << std::endl;
    }

    // 返回引擎
    return engine;
}

// 主函数
int main() {
    std::string engineFile = "/home/tam/wk/vpr/model/zhongda/mix_512.onnx.engine"; // 引擎文件路径
    nvinfer1::ICudaEngine* engine = loadEngine(engineFile);

    if (!engine) {
        std::cerr << "Failed to load engine" << std::endl;
        return -1;
    }

    // 引擎加载成功
    std::cout << "Engine loaded successfully!" << std::endl;

    // 创建推理上下文
    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
    if (!context) {
        std::cerr << "Failed to create execution context" << std::endl;
        engine->destroy();
        return -1;
    }

    // 假设我们有输入数据输入到GPU内存
    float* inputData = nullptr;  // 输入数据指针
    float* outputData = nullptr; // 输出数据指针
    size_t inputSize = 1 * 3 * 224 * 224 * sizeof(float); // 假设输入是 3x224x224 的图像
    size_t outputSize = 1 * 1000 * sizeof(float); // 假设输出是 1000 类别

    // 分配CUDA内存
    cudaMalloc((void**)&inputData, inputSize);
    cudaMalloc((void**)&outputData, outputSize);

    // 设置输入数据 (根据模型输入的具体要求来填充)

    // 执行推理
    void* buffers[] = { inputData, outputData };
    context->executeV2(buffers);

    // 获取结果并进行处理
    std::vector<float> output(1000);
    cudaMemcpy(output.data(), outputData, outputSize, cudaMemcpyDeviceToHost);

    // 打印输出结果
    for (size_t i = 0; i < 10; ++i) {  // 打印前10个预测结果
        std::cout << "Class " << i << ": " << output[i] << std::endl;
    }

    // 清理资源
    cudaFree(inputData);
    cudaFree(outputData);
    context->destroy();
    engine->destroy();

    return 0;
}
