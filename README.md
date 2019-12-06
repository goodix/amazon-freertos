# Getting Started Guide for Goodix GR5515-SK

[TOC]

## 1. Introduction

This tutorial provides instructions on getting started with the Amazon FreeRTOS system based on Goodix GR5515 Starter Kit Board (here after referred to as "Goodix GR5515-SK Board"). If you do not have the Goodix GR5515-SK Board, visit the [Goodix](https://www.goodix.com/en/support/distributors) to purchase one . 

Before getting started, you need to [Set Up AWS IoT and Amazon Cognito for Amazon FreeRTOS Bluetooth Low Energy](https://docs.aws.amazon.com/freertos/latest/userguide/ble-demo.html). 

To run the Amazon FreeRTOS Bluetooth Low Energy demo, you also need an iOS or Android mobile device with Bluetooth and Wi-Fi capabilities. 

**Note:** 

> GR5515-SK Board uses the SDK provided by Amazon to route to AWS Cloud, and the SDK is available in both iOS and Android versions. You can clone source codes from: 
>
> - iOS: https://github.com/aws/amazon-freertos-ble-ios-sdk
> - Android: https://github.com/aws/amazon-freertos-ble-android-sdk
>
> If you are using an iOS mobile device, you need Xcode to build a demo mobile application. If you are using an Android mobile device, you can use Android Studio to build a demo mobile application.
>



## 2. Set Up the Goodix Hardware

1. Hardware preparation:
   - A GR5515-SK Board
   - A Micro USB cable



## 3. Set UP the Development Environment

The following instructions apply to Windows system. To build an Amazon-FreeRTOS development environment for GR5515-SK Board, the following software is required: 

- Keil µVision5: Integrated Development Environment (IDE) used by GR5515-SK Board
- Device Family Pack (DFP) that supports ARM Cortex-M4 FPU Chip 
- J-Link: programming and debugging tool for GR5515-SK Board
- Serial port assistant tool: log printout
- GR551x family chip programming algorithm file: the GR551x family chip programming algorithm file in Keil
- GNU Make and CMake: tools to build projects through command lines



### 3.1 Install Keil µVision5

1. Click to download the right version of installation program for Arm Cortex-M from the Keil website; recommended version: µVision5 V5.20. 

   It is available at: http://www2.keil.com/mdk5/legacy. 

2. Run the downloaded program and choose the default installation path to install the Keil µVision5. 

### 3.2 Install ARM CM4-FP DFP

1. Click to download the installation program of ARM Cortex-M4 FP DFP from the Keil website, and save it to an appropriate directory. 

   It is available at: http://www.keil.com/dd2/arm/armcm4_fp/. 

2. Run Keil; on the file toolbar, find the button **Pack Installer** and click it. 

3. From the menu, click **File** > **Import**, to import the downloaded DFP file and finish installation. The interface after installation is shown as follows: 

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_3_1.png)

### 3.3 Install J-Link

GR5515-SK Board provides on-board JTAG/SWD debugging interface, and you only need to install the J-Link software, instead of purchasing a J-Link emulator. 

1. Download the J-Link software. 

   It is available at: https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPackBeta. 

   J-Link software version: V6.10 or later versions are required, and the latest version is recommended. 

2. After downloading, click to install the software in the default path. 

### 3.4 Install GR5515-SK Board Serial Port Assistant Tool

1. Download an appropriate serial port assistant tool from the Internet and install it. 

### 3.5 Install GR551x Family Chip Algorithm File

1. Download the GR551x family chip algorithm file. 

   It is available at: [GR551x_8Mb_Flash.FLM](https://github.com/goodix/amazon-freertos/tree/goodix/vendors/goodix/GR551x_SDK_V1_00/build/binaries/GR551x_8MB_Flash.FLM). 

2. After downloading, save it to the Flash directory in the installation path of Keil. 

   Default path: C:\Keil_v5\ARM\Flash

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_3_2.png)



### 3.6 Install GNU Make and CMake

If you use Keil IDE for project development only, you do not need to install GNU Make and CMake; if you want to use CMake command line to build projects, you need to install them. 

1. Download GNU Make. 

   It is available at: ftp://ftp.equation.com/make/. 

2. GNU Make is an installation-free executable file. Save it to an appropriate path, such as c:\gnumake\, and then add it to the environment variable path. 

3. Download and install CMake. 

   Install CMake in the default path.

   CMake (V3.13 or later versions) is available at: https://cmake.org/. 
   
   

## 4. Configure and Build Amazon FreeRTOS Project

### 4.1 Download Amazon FreeRTOS

To download Amazon FreeRTOS for Goodix GR5515-SK Board, go to the [Amazon FreeRTOS GitHub page](https://github.com/goodix/amazon-freertos) and clone the repository. 

**Note:**

> The maximum length of a file path on Microsoft Windows is 260 characters. Lengthy Amazon FreeRTOS download directory paths can cause project build failures. 

> In this guide, the path to the amazon-freertos directory is referred to as $(amazon-freertos)  . 

### 4.2 Connect Device

1. Run the Keil µVision5, and open the project  $(amazon-freertos)\projects\goodix\GR5515-SK\keil_v5\aws_demos\goodix_aws_demos.uvprojx.  

2. You might be prompted to choose a chip type when you open the project for the first time. If you are prompted, choose **ARM Cortex M4** > **ARMCM4_FP**; if this chip type is not available in the pop-up box, see **3.2 Installing ARM CM4-FP DFP** to install required pack files. 

3. The project directory structure is shown as follows: 

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_aws_demo_4_1.png)

   Project files are organized by (1) **Main entry and hardware initialization**, (2) **Demo and related header files**, (3) **AWS FreeRTOS public dependency**, (4) **GR5515 SDK dependency**, and (5) **Porting files and configuration of each module**, as shown in the above figure, to help users locate files quickly. 

4. Use a Micro USB cable to connect a PC to GR5515-SK Board; check whether any device is discovered through **Device Manager** of Windows, as shown below. (Note: The serial port number of different computers may vary.)

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_2.png)

5. Choose the project and right click the menu: (1) choose **Options for Target 'aws_demos'**; (2) choose **Debug** tab; (3) choose **J-LINK/J-TRACE Cortex** from the **Use** list; (4) click **Settings**; (5) choose **Debug** tab; (6) choose **SW** under **Port**, as shown in the following figure. Verify that the SW device is available. 

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_3.png)

6. In the **Cortex/JLink/JTrace Target Driver Setup** pop-up, choose **Flash Download**, choose **GR551x 8 Mb Flash**, and click **Add** to set up algorithm file for GR551x family chip. 

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_4_4.png)

7. Open the serial port assistant tool, choose GR5515-SK Board port (configuration parameter: 115200|8|N|1), and open the port. 

### 4.3 Configure AWS IoT Endpoint

1. Browse to the [AWS IoT console](https://console.aws.amazon.com/iotv2/). 

2. In the navigation pane, choose **Settings**. 

   Your AWS IoT endpoint is displayed in **Endpoint**. It should be similar to `*<1234567890123>*-ats.iot.*<us-east-1>*.amazonaws.com`. Record this endpoint. 

3. In the navigation pane, choose **Manage** > **Things**. Your device should have an AWS IoT thing name. Record this name. 

4. Use your AWS IoT endpoint and AWS IoT thing name to open `$(amazon-freertos)/demos/include/aws_clientcredential.h` in IDE, and specify values for the following `#define` constants: 

   - `clientcredentialMQTT_BROKER_ENDPOINT` *Your AWS IoT endpoint*
   - `clientcredentialIOT_THING_NAME` *The AWS IoT thing name of your board*

### 4.4 Build a Project

1. In the menu, click **Project** > **Build Target** to compile a project. After compilation, build a target binary file in the directory $(amazon-freertos)\projects\goodix\GR5515-SK\keil_v5\aws_demos\build. 
2. To compile a project with make + CMake, see **6. Building a Project with CMake**. 



## 5. Enable Project Demo

GR5515-SK Board supports the following demo projects: 

- CONFIG_MQTT_DEMO_ENABLED: demo project that establishes network connection to AWS IoT Cloud through MQTT (over BLE) for data **Subscribe/Publish** demos
- CONFIG_BLE_GATT_SERVER_DEMO_ENABLED: GATT service demo project of Bluetooth Low Energy
- CONFIG_OTA_UPDATE_DEMO_ENABLED: demo project that updates application through OTA

GR5515-SK Board project configures other macro functions by default. Users can follow instructions in **4.3 Configuring Your AWS IoT Endpoint** to configure an endpoint, and then define above macros (users can define one macro each time) in the file $(amazon-freertos)\vendors\goodix\boards\GR5515-SK\aws_demos\config_files\aws_demo_config.h, to enable the demonstration function of corresponding demos. 

1. Compile the project after enabling macro definitions of corresponding demos. 
2. After compilation, in the menu, click **Flash** > **Download** (or **F8**) for programming. 
3. After programming, start the demonstration program by pressing the **RESET** button on GR5515-SK Board, or use the **Debug** menu of Keil software for project debugging. 

For details about demo, see:

- [MQTT over Bluetooth Low Energy](https://docs.aws.amazon.com/freertos/latest/userguide/ble-demo.html#ble-demo-mqtt) 
- [Generic Attributes Server](https://docs.aws.amazon.com/freertos/latest/userguide/ble-demo.html#ble-demo-server) 

- [Over-the-Air Updates](https://docs.aws.amazon.com/zh_cn/freertos/latest/userguide/freertos-ota-dev.html)



## 6. Build a Project with CMake

**Note:**

> CMake version shall meet the Amazon FreeRTOS requirements. 

### 6.1 CMake Script Dependency

- CMake file path of goodix/GR5515-SK: $(amazon-freertos)\\vendors\goodix\boards\GR5515-SK\CMakeLists.txt
- CMake compiler file of goodix/GR5515-SK: $(amazon-freertos)\\tools\cmake\toolchains\arm-keil-v5.cmake

### 6.2 Use CMake Command Line for Compilation

1. Compile **Demo** project: 

```cmake
cd {SRC_ROOT}
cmake -DVENDOR=goodix -DBOARD=GR5515-SK -DCOMPILER=arm-keil-v5 -DAFR_ENABLE_TESTS=0 -S . -B build -DAFR_METADATA_MODE=1 -G "Unix Makefiles"

cd build
make -j8
```

2. Compile **Test** project: 

```cmake
cd {SRC_ROOT}
cmake -DVENDOR=goodix -DBOARD=GR5515-SK -DCOMPILER=arm-keil-v5 -DAFR_ENABLE_TESTS=1 -S . -B build -DAFR_METADATA_MODE=1 -G "Unix Makefiles"

cd build
make -j8
```

3. CMake command execution is defined as follows: 

```
-- The C compiler identification is ARMCC 5.6.61
-- The CXX compiler identification is ARMCC 5.6.61
-- The ASM compiler identification is ARMCC
-- Found assembler: C:/Keil_v5/ARM/ARMCC/bin/armasm.exe
-- Found Git: C:/Program Files/Git/cmd/git.exe (found version "2.22.0.windows.1")
=========================Resolving dependencies==========================
module disabled: wifi
reason: wifi::mcu_port is not defined by vendor.
dependency path: ble_wifi_provisioning->wifi->wifi::mcu_port

module disabled: secure_sockets
reason: secure_sockets::mcu_port is not defined by vendor.
dependency path: greengrass->secure_sockets->secure_sockets::mcu_port


====================Configuration for Amazon FreeRTOS====================
  Version: 201906.00 Major
  Git version: Unknown

Target microcontroller:
  vendor: Goodix
  board: GR5515-SK
  description: Goodix BLE GR5515-SK Board for AmazonFreeRTOS
  family: Goodix BLE GR551x
  data ram size: 256KB
  program memory size: 1MB

Host platform:
  OS: Windows-10.0.17134
  Toolchain: arm-keil-v5
  Toolchain path: C:/Keil_v5/ARM/ARMCC
  CMake generator: Unix Makefiles

Amazon FreeRTOS modules:
  Modules to build: ble, common, crypto, dev_mode_key_provisioning, kernel, mqtt,
                           ota, pkcs11, platform, posix, serializer, shadow
  Enabled by user: ble, mqtt, ota, pkcs11, platform, posix, shadow
  Enabled by dependency: ble_hal, common, crypto, demo_base, dev_mode_key_provisioning,
                           freertos, freertos_plus_posix, kernel, serializer
  3rdparty dependencies: jsmn, mbedtls, pkcs11, tinycbor
  Available demos: demo_ble, demo_mqtt, demo_ota, demo_shadow
  Available tests:
=========================================================================

-- Configuring done
-- Generating done
```

### 6.3 Use CMake-GUI for Compilation

1. Open CMake-GUI, configure the source code and build paths, and click **Configure**. 

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_1.jpg)

2. Choose the makefile generator in the pop-up. 

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_2.jpg)

2. Choose the CMake compiler script. 

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_3.jpg)

3. Finish other configurations. Choose **goodix.GR5515-SK**. To compile Demo project, do not check **AFR_ENABLE_TESTS**; to compile Test project, check **AFR_ENABLE_TESTS**. 

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_5.jpg)

4. After configuration, change to the build directory from command line, enter the directory, and execute the **make** command. 

   ![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/aws_keil_9.png)



## 7. Debugging

Keil provides a debug menu with rich functions. When you perform runtime debugging for GR5515-SK Board with J-Link tool, common debugging functions are shown as follows: 

![](https://github.com/goodix/amazon-freertos/raw/goodix/vendors/goodix/refs/gr5515_cmake_6_6.png)



## 8. Troubleshooting

For general troubleshooting information about getting started with Amazon FreeRTOS, see [Troubleshooting Getting Started](https://docs.aws.amazon.com/freertos/latest/userguide/gsg-troubleshooting.html). 

