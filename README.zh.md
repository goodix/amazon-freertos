# Amazon-FreeRTOS基于Goodix GR5515-SK入门指南

[TOC]

[EN](https://github.com/goodix/amazon-freertos/blob/goodix/README.md)   |  [中文](https://github.com/goodix/amazon-freertos/blob/goodix/README.zh.md)

## 1. 引导

本教程将介绍基于Goodix GR5515-SK开发板的Amazon FreeRTOS系统入门。如果您没有此开发板，可以点击 [这里](https://www.goodix.com/en/support/distributors) 购买一块。

在开始之前，您需要[为 Amazon FreeRTOS 低功耗蓝牙设置 AWS IoT 和 Amazon Cognito](https://docs.aws.amazon.com/zh_cn/freertos/latest/userguide/ble-demo.html#set-up-ble-demo-aws)。

要运行 Amazon FreeRTOS 低功耗蓝牙，您还需要具有蓝牙和 Wi-Fi 功能的 iOS 或 Android 移动设备。

**注意**

> GR5515-SK开发板使用Amazon官方提供的SDK 作为接入AWS Cloud的路由, 提供了iOS版本和Android版本,您可以从这里clone源码:
>
> - iOS版本源码 :  https://github.com/aws/amazon-freertos-ble-ios-sdk
> - Android 版本源码: https://github.com/aws/amazon-freertos-ble-android-sdk
>
> 如果您使用的是 iOS 设备，则需要 Xcode 来构建移动应用程序。如果您使用的是 Android 设备，则可使 用 Android Studio 来构建移动应用程序。



## 2. 设置硬件

1. 硬件准备:
   - 一块GR5515-SK开发板
   - 一条Micro USB接口线



## 3. 配置开发环境

配置说明主要针对Windows系统，基于GR5515-SK开发板进行Amazon-FreeRTOS开发，需要依赖如下软件和文件：

- Keil μVision5：GR5515-SK使用的集成开发环境
- 支持 ARM Cortex-M4 FPU芯片的DFP（Device Family Pack）软件包 
- JLink：可用于GR5515-SK开发板的程序烧写和调试
- 串口助手工具：用于日志的打印输出
- GR5515芯片烧写算法文件：Keil下GR5515芯片的烧写算法文件
- GnuMake和cmake工具：适用于通过命令行构建工程



### 3.1 安装集成开发环境 Keil μVision5

1. 在Keil软件官网点击下载适用于Arm Cortex-M的安装程序，推荐下载版本：μVision V5.20。

   下载地址： http://www2.keil.com/mdk5/legacy

2. 运行下载的安装程序，进行Keil软件的安装，安装路径选择默认即可。

### 3.2 安装ARM CM4-FP DFP

1. 在Keil软件官网点击下载Arm cortex-m4 fp dfp安装程序，保存到合适的目录。

   下载地址：http://www.keil.com/dd2/arm/armcm4_fp/

2. 启动Keil 软件，在文件工具栏找到按钮Pack Installer，点击打开。

3. 点击菜单 File-> Import，导入下载的dfp文件，完成安装。安装完成后的图片如下所示。

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_3_1.png)

### 3.3 安装JLink

GR5515-SK开发板已板载支持JTAG/SWD调试接口，不需要再购买JLink仿真器，只需要安装JLink软件即可。

1. 下载JLink软件。

   下载地址： https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPackBeta

   JLink软件版本要求：6.10+，建议下载最新版。

2. 下载完成后，点击安装到默认路径即可。

### 3.4 安装GR5515-SK串口助手工具

1. 在网络下载合适的串口助手工具进行安装。

### 3.5 安装GR5515芯片算法文件

1. 下载GR5515芯片算法文件。

   下载地址：[GR551x_8Mb_Flash.FLM](https://github.com/goodix/amazon-freertos/tree/goodix/vendors/goodix/GR551x_SDK_V1_00/build/binaries/GR551x_8MB_Flash.FLM)

2. 文件下载完成后，存放到Keil安装路径下Flash目录。

  一般默认路径为：C:\Keil_v5\ARM\Flash

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_3_2.png)



### 3.6 安装gnu-make和cmake

如果只使用keil集成环境进行项目开发，可以不用安装gnu-make和cmake软件；如果希望使用cmake命令行进行项目构建，需要安装gnu-make和cmake。

1. 下载gnu-make。

   下载地址：ftp://ftp.equation.com/make/

2. gnu-make为非安装可执行文件，放置到合适的路径，如c:\gnumake\，然后加入到环境变量path。

3. 下载cmake后点击安装。

   安装在默认路径下。

   cmake（版本>3.13），下载地址：https://cmake.org/
   
   

## 4. 配置和构建Amazon FreeRTOS项目

### 4.1 下载Amazon FreeRTOS

要下载适用于Goodix GR5515-SK的Amazon FreeRTOS，请转至Goodix的Github组织账户下 项目[Amazon FreeRTOS ](https://github.com/goodix/amazon-freertos)并克隆存储库。

**注意**

> Microsoft Windows 上的文件路径最大长度为260个字符。过长的 Amazon FreeRTOS下载目录路径可能会导致构建操作失败。

> 在本教程中，amazon-freertos目录的路径称为 $(amazon-freertos)。

### 4.2 建立设备连接

1. 开启keil μVision5软件，打开工程 $(amazon-freertos)\projects\goodix\GR5515-SK\keil_v5\aws_demos\goodix_aws_demos.uvprojx 

2. 第一次打开工程，可能会提示选择芯片型号。如有提示，请选择ARM Cortex M4 -> ARMCM4_FP，如弹出的芯片选择框中无此芯片型号，请参考 **3.2 安装ARM CM4-FP DFP** 安装相关pack文件。

3. 打开后的工程目录结构：

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_aws_demo_4_1.png)

   工程按照“main入口及硬件初始化 | Demo及相关头文件 | aws freertos公共依赖 | GR5515 SDK依赖 | 各模块Porting文件及配置”的分类进行文件组织，便于用户快速定位文件。

4. 使用Micro-USB线连接PC和GR5515-SK开发板，可通过Windows系统设备管理器查看是否找到设备，如图(注意， 不同电脑串口号可能不一致)。

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_2.png)

5. 选中工程，右键选择菜单：Options for Target -> 选中Debug选项卡 -> 选中USE下拉菜单 JLINK/JTRACE-Cortex ->点击Settings -> 选中Debug选项卡 -> Port 选择 SW，如下图。确认找到芯片设备。

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_3.png)

6. 如下图，在Cortex/JLink/JTrace Target Driver Setup 弹窗选择选项卡 Flash Download，可通过 Add 按钮选择并设置GR551x芯片的下载算法文件。

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_4.png)

7. 打开串口助手工具，选择GR5515-SK开发板串口，配置参数：115200|8|N|1， 然后开启串口。

### 4.3 配置 AWS IoT 终端节点

1. 浏览至 [AWS IoT 控制台](https://console.aws.amazon.com/iotv2/)。

2. 在导航窗格中，选择 **Settings**。

   您的 AWS IoT 终端节点显示在 **Endpoint (终端节点)** 文本框中。它应该类似于 `*<1234567890123>*-ats.iot.*<us-east-1>*.amazonaws.com`。记下此终端节点。

3. 在导航窗格中，选择**管理**，然后选择**事物**。记下设备的 AWS IoT 事物名称。

4. 利用您拥有的 AWS IoT 终端节点和 AWS IoT 事物名称，在 IDE 中打开 `$(amazon-freertos)/demos/include/aws_clientcredential.h`，并为以下 `#define` 常量指定值：

   - `clientcredentialMQTT_BROKER_ENDPOINT` *您的 AWS IoT 终端节点*
   - `clientcredentialIOT_THING_NAME` *您的主板的 AWS IoT 事物名称*

### 4.4 进行项目构建

1. 点击菜单 Project -> Build Target，可进行工程的编译， 编译完成后，会在 $(amazon-freertos)\projects\goodix\GR5515-SK\keil_v5\aws_demos\build 目录生成目标二进制文件。
2. 如果使用make + cmake 进行项目的编译，请参考章节：**6. 通过cmake构建项目**。



## 5. 启用项目演示

GR5515-SK支持如下演示项目：

- CONFIG_MQTT_DEMO_ENABLED: 通过 MQTT ( over BLE )和AWS Iot Cloud建立网络连接，进行数据Subscribe/Publish演示的项目
- CONFIG_BLE_GATT_SERVER_DEMO_ENABLED: BLE低功耗蓝牙的GATT服务演示项目

默认情况下，GR5515-SK工程已完成了其他宏功能的配置，用户遵照 4.3 章节配置好终端节点后，即可以在文件 $(amazon-freertos)\vendors\goodix\boards\GR5515-SK\aws_demos\config_files\aws_demo_config.h中通过定义如上宏（单次只允许定义一个宏），开启相应Demo的演示功能。

1. 启用相应演示项目宏定义后，进行项目的编译。
2. 项目编译完成后，点击菜单 Flash -> Download （或者快捷键 F8），进入程序的烧录。
3. 程序烧录成功后，可通过开发板复位键启动演示程序，或者使用Keil软件的Debug菜单，进入项目的调试。

关于演示的更多细节请参考：

- [MQTT over Bluetooth Low Energy](https://docs.aws.amazon.com/freertos/latest/userguide/ble-demo.html#ble-demo-mqtt) 
- [Generic Attributes Server](https://docs.aws.amazon.com/freertos/latest/userguide/ble-demo.html#ble-demo-server) 



## 6. 通过cmake构建项目

**注意**

> cmake版本和Amazon FreeRTOS要求保持一致。

### 6.1 cmake 脚本依赖

- goodix/GR5515-SK 的cmake文件路径:  $(amazon-freertos)\vendors\goodix\boards\GR5515-SK\CMakeLists.txt
- goodix/GR5515-SK 的cmake编译器文件:  $(amazon-freertos)\tools\cmake\toolchains\arm-keil-v5.cmake

### 6.2 使用cmake 命令行编译

1. 编译Demo工程：

```cmake
cd {SRC_ROOT}
cmake -DVENDOR=goodix -DBOARD=GR5515-SK -DCOMPILER=arm-keil-v5 -DAFR_ENABLE_TESTS=0 -S . -B build -DAFR_METADATA_MODE=1 -G "Unix Makefiles"

cd build
make -j8
```

2. 编译Test 工程：

```cmake
cd {SRC_ROOT}
cmake -DVENDOR=goodix -DBOARD=GR5515-SK -DCOMPILER=arm-keil-v5 -DAFR_ENABLE_TESTS=1 -S . -B build -DAFR_METADATA_MODE=1 -G "Unix Makefiles"

cd build
make -j8
```

3. cmake 命令执行示意：

```
-- The C compiler identification is ARMCC 5.6.61
-- The CXX compiler identification is ARMCC 5.6.61
-- The ASM compiler identification is ARMCC
-- Found assembler: C:/Keil_v5/ARM/ARMCC/bin/armasm.exe
-- Found Git: C:/Program Files/Git/cmd/git.exe (found version "2.22.0.windows.1")
=========================Resolving dependencies==========================
module disabled: wifi
reason:          wifi::mcu_port is not defined by vendor.
dependency path: ble_wifi_provisioning->wifi->wifi::mcu_port

module disabled: secure_sockets
reason:          secure_sockets::mcu_port is not defined by vendor.
dependency path: greengrass->secure_sockets->secure_sockets::mcu_port


====================Configuration for Amazon FreeRTOS====================
  Version:                 201906.00 Major
  Git version:             Unknown

Target microcontroller:
  vendor:                  Goodix
  board:                   GR5515-SK
  description:             Goodix BLE GR5515-SK Board for AmazonFreeRTOS
  family:                  Goodix BLE GR551x
  data ram size:           256 KB
  program memory size:     1 MB

Host platform:
  OS:                      Windows-10.0.17134
  Toolchain:               arm-keil-v5
  Toolchain path:          C:/Keil_v5/ARM/ARMCC
  CMake generator:         Unix Makefiles

Amazon FreeRTOS modules:
  Modules to build:        ble, common, crypto, dev_mode_key_provisioning, kernel, mqtt,
                           ota, pkcs11, platform, posix, serializer, shadow
  Enabled by user:         ble, mqtt, ota, pkcs11, platform, posix, shadow
  Enabled by dependency:   ble_hal, common, crypto, demo_base, dev_mode_key_provisioning,
                           freertos, freertos_plus_posix, kernel, serializer
  3rdparty dependencies:   jsmn, mbedtls, pkcs11, tinycbor
  Available demos:         demo_ble, demo_mqtt, demo_ota, demo_shadow
  Available tests:
=========================================================================

-- Configuring done
-- Generating done
```

### 6.3 使用cmake-gui编译

1. 打开CMake-GUI，配置源码和build路径，然后点击Configure。

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_1.jpg)

2. 在弹窗中选择makefile generator。

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_2.jpg)

3. 选择cmake编译器脚本。

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_3.jpg)

4. 完成其他配置。选择goodix.GR5515-SK。如果编译demo工程，不勾选AFR_ENABLE_TESTS；如果编译test工程，勾选AFR_ENABLE_TESTS。

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_5.jpg)

5. 上述配置完成后，在命令行切换到 build 目录，进入build目录，执行make命令

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_9.png)



## 7. 调试

Keil提供了丰富的调试菜单，通过JLink工具进行GR5515-SK的运行时调试，常用的调试功能：

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_6.png)



## 8. 故障排除

有关Amazon FreeRTOS入门的常规故障排除信息，请参阅[问题排查入门](https://docs.aws.amazon.com/zh_cn/freertos/latest/userguide/gsg-troubleshooting.html)。

