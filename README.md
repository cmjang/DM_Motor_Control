# 达妙电机Linux-C++驱动库
该驱动库对达妙客户提供的驱动库进行了魔改，增加了诸多功能。该驱动库目前只能在Linux下使用，特别适合使用ROS的宝宝

欢迎加入QQ群：677900232 进行达妙电机技术交流

## 1.引入达妙库

通过头文件引入达妙库

```c++
#include "damiao.h"
```

## 2.定义电机控制对象

对电机进行控制需要定义相关的对象。

首先是串口对象，达妙的USB转串口默认是921600波特率，然后串口号的选择大家根据自己linux设备下的串口号进行选择。

```c++
auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
```

然后要定义相关的电机控制对象和电机对象。其中电机对象中第一个参数是电机类型需要进行选择，第二个参数是电机的SlaveID即CANID即电机控制的ID，**第三个参数是MasterID即主机ID，主机ID非常不建议为0x00，建议设置为CANID基础上＋0x10，例如下面的0x11.**

```c++
auto dm =damiao::Motor_Control(serial);
damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4310,0x05, 0x15);
```

## 3.使能电机

使能电机之前需要先进行添加电机操作

```c++
dm.addMotor(&M1);
dm.addMotor(&M2);
```

然后使能电机

```c++
dm.enable(M1);
dm.enable(M2);
```

## 4.电机控制

目前驱动库支持4种控制模式。

推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。

### 4.1 MIT控制模式

默认的control是mit控制模式，第一个参数电机对象，第二个是kp，第三个是kd，第四个是位置，第五个是速度，第六个是扭矩

```c++
dm.control(M1, 50, 0.3, 0, 0, 0);
```

### 4.2 位置速度模式

第一个参数是电机对象，第二个参数是对应的位置，第三个参数是旋转到该位置用的速度

```c++
float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
dm.control_pos_vel(M2, q*10, 5);
```

### 4.3速度模式

第二参数就是电机的转速都为达妙所使用的标准单位

```c++
dm.control_vel(M1, q*10);
```

### 4.4力位混合模式

第一个是电机对象，第二个是电机位置，第三个是电机速度范围是0-10000，第四个是电机电流范围为0-10000。具体详细请查看达妙文档

```c++
dm.control_pos_force(M1,5,500, 1000)
```

## 5.电机模式更改

达妙电机新固件支持使用can进行电机模式修改，以及修改其他参数等操作。具体请咨询达妙客服。

通过下面的函数可以对电机的控制模式进行修改。支持四种控制模式在线修改如下四种MODE：

MIT_MODE ,

POS_VEL_MODE,

VEL_MODE,

POS_FORCE_MODE

下面是修改的demo。

```c++
dm.switchControlMode(M1, damiao::VEL_MODE);
dm.switchControlMode(M2, damiao::POS_VEL_MODE);
```

**保存参数**

默认电机修改模式等操作后参数不会保存到flash中，需要使用命令如下进行保存至电机的flash中

```c++
  dm.save_motor_param(M1);
  dm.save_motor_param(M2);
```

