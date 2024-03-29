# OrbbecSDK
<!-- -->
The Orbbec 3D camera product software development kit，fully supports UVC, realizes driver-free plug-and-play, provides low-level and high-level simple and easy-to-use APIs, and helps developers use it flexibly in different scenarios.

Besides, this SDK is compatible with Orbbec's original OpenNI protocol devices through built-in code, so that developers can completely migrate to OrbbecSDK to support Orbbec's new generation of products and old products with one set of code.

## what is include in the repository

* Doc
> API reference documentation and sample documentation.

* Driver
> Windows driver (DaBai, DaBai DCW, DaBai DW, Astra mini Pro, Astra Pro Plus, A1 Pro, Gemini E, Gemini E Lite, Gemini These device that use OpenNI's  protocol, and need to install driver on Windows, while device using standard UVC protocols do not need to install driver)

* Script
> Windows: timestamp registration script, used to solve the problem of obtaining UVC timestamps and metadata under Windows.

> Linux: udev rules installation scripts to resolve device permission issues.

* Example
> C/C++ samples, including sample project source code and compiled executables.

* SDK
> OrbbecSDK header files and library files, OrbbecSDK running configuration files

## Platform support

* Windows：Windows 10 (x64)
* Linux: Ubuntu 16.04/18.04/20.04/22.04 (x64)
* Arm32: Ubuntu16.04/18.04/20.04/22.04
* Arm64: Ubuntu18.04/20.04/22.04
Note: supported Arm platforms: jestson nano (arm64), A311D (arm64), Raspberry Pi 4 (arm64), Raspberry Pi 3 (arm32), rk3399 (arm64), other Arm systems, may need to Cross-compile.
* Mac: M series chip 13.0 and above os systems.

## Product support

| **products list** | **firmware version** |
| --- | --- |
| Femto Bolt       | 1.0.6  (unsupported ARM32) |
| Gemini 2 XL     | Obox: V1.2.5  VL:1.4.54 |
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
| Gemini E Lite  | 3606                  |
| Gemini         | 3.0.18                      |
| Astra Mini S Pro | 1.0.05                      |

## Getting started

* Get source code

```
https://github.com/orbbec/OrbbecSDK.git
```

* How to Compile Sample

Please refer to [Environment_Configuration.md ](Doc\tutorial\English\Environment_Configuration.md)

* Use OrbbecSDK in your CMake project

``` cmake
cmake_minimum_required(VERSION 3.1.15)
project(OrbbecSDKTest)
find_package(OrbbecSDK REQUIRED)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} OrbbecSDK::OrbbecSDK)
```

* Ready to start

Our library offers a high level API for using Orbbec depth cameras. The following snippet shows how to start streaming frames and extracting the depth value of a pixel:

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

For more information on the library, please follow our [examples](Example\README.md).

## Related links

* [Orbbec 3D Club](https://3dclub.orbbec3d.com)
* [Download OrbbecSDK](https://www.orbbec.com/developers/orbbec-sdk/)