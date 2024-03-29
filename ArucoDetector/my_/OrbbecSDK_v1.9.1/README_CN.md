# OrbbecSDK
<!-- 本文档面向开发者 -->
新一代 Orbbec 3D 相机产品软件开发套件，全面支持 UVC 协议，实现免驱即插即用，提供低阶和高阶简洁、易用的 API，帮助开发者在不同场景下灵活使用。

## 目录说明

1.Doc
> API 参考文档及示例说明文档。

2.Driver
> windows device 驱动程序，只有windows的部分模组需要安装。（DaBai、DaBai DCW、DaBai DW、Astra mini Pro、Astra Pro Plus、A1 Pro、Gemini E、Gemini E Lite、Gemini 这些使用的 OpenNI 的私有协议的模组，在 windows 上需要安装私有的驱动，而使用标准 UVC 协议的模组，不需要安装驱动）。

3.Script
> Windows：时间戳注册脚本，用于解决 Windows 下获取 UVC 时间戳及 Metadata 问题。
> Linux：udev rules 安装脚本，用于解决设备访问权限问题。

4.Example
> C/C++ 示例，包括示例工程源码和已编译好的可执行文件。

5.SDK
> OrbbecSDK 头文件和库文件、OrbbecSDK运行配置文件。

## 操作系統

* Windows：Windows 10 (x64)
* Linux: Ubuntu 16.04/18.04/20.04/22.04 (x64)
* Arm32: Ubuntu16.04/18.04/20.04/22.04
* Arm64: Ubuntu18.04/20.04/22.04
注：当前版本支持的Arm平台：jestson nano(arm64)、A311D(arm64)、树莓派4(arm64)、树莓派3（arm32）、rk3399(arm64), 其它Arm系统，可能需要重新交叉编译
* Mac：  M1和M2系列芯片 13.0及以上系统。

## 入门指南
* 获取源码
```
https://github.com/orbbec/OrbbecSDK.git
```

* 编译Sample
Please refer to [Environment_Configuration.md ](Doc\tutorial\Chinese\Environment_Configuration.md)


* Depth示例
以下代码演示如何获取Depth及打印中心点的Depth值。

```
    // Create a pipeline with default device
    ob::Pipeline pipe;

    auto profiles = pipe.getStreamProfileList(OB_SENSOR_DEPTH);
    auto depthProfile = std::const_pointer_cast<ob::StreamProfile>(profiles->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();

    // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
    config->enableStream(depthProfile);

    // Start the pipeline with config
    pipe.start(config);

    while(true) {
        // Wait for up to 100ms for a frameset in blocking mode.
        auto frameSet = pipe.waitForFrames(100);
        if(frameSet == nullptr) {
            continue;
        }

        auto depthFrame = frameSet->depthFrame();

        // for Y16 format depth frame, print the distance of the center pixel every 30 frames
        if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
            uint32_t  width  = depthFrame->width();
            uint32_t  height = depthFrame->height();
            uint16_t *data   = (uint16_t *)depthFrame->data();

            float centerDistance = data[width * height / 2 + width / 2];
            std::cout << "center distance " << centerDistance << std::endl;
        }
    }

    // Stop the pipeline
    pipe.stop();

```
更多的Sample用例，请参考 [examples](Example\README_CN.md).

## 支持的硬件产品

| **产品列表** | **固件版本** |
| --- | --- |
| Femto Bolt       | 1.0.6  (unsupported ARM32) |
| Gemini 2 XL     | Obox: V1.2.5  VL:1.4.54  |
| Astra 2         | 2.8.20                    |
| Gemini 2 L      | 1.4.32                     |
| Gemini 2        | 1.4.60 /1.4.76                    |
| Femto Mega      | 1.1.7  (window10、ubuntu20.04、ubuntu22.04)                     |
| Astra+         | 1.0.22/1.0.21/1.0.20/1.0.19 |
| Femto          | 1.6.7                       |
| Femto W       | 1.1.8          |
| DaBai          | 2436                        |
| DaBai DCW      | 2460                        |
| DaBai DW       | 2606                        |
| Astra Mini Pro | 1007                        |
| Gemini E       | 3460                        |
| Gemini E Lite  | 3606                        |
| Gemini         | 3.0.18                      |
| Astra Mini S Pro | 1.0.05                    |

## 相关链接

* [3D视觉开发者社区](https://developer.orbbec.com.cn/)
* [OrbbecSDK主页](https://developer.orbbec.com.cn/develop_details.html?id=1)

