# 达妙电机Linux-C++驱动库
该驱动库对达妙客户提供的驱动库进行了魔改，增加了诸多功能。最低支持的C++版本为C++11

该驱动库目前只能在Linux下使用，特别适合使用ROS的宝宝

**欢迎加入QQ群：677900232 进行达妙电机技术交流。欢迎进入达妙店铺进行点击选购**[首页-达妙智能控制企业店-淘宝网 (taobao.com)](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

## 1.引入达妙库

​	通过头文件引入达妙库

```c++
#include "damiao.h"
```

## 2.定义电机控制对象

​	对电机进行控制需要定义相关的对象。

​	首先是串口对象，达妙的USB转串口默认是921600波特率，然后串口号的选择大家根据自己linux设备下的串口号进行选择。

```c++
auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
```

​	然后要定义相关的电机控制对象和电机对象。其中电机对象中第一个参数是电机类型需要进行选择，第二个参数是电机的SlaveID即CANID即电机控制的ID，**第三个参数是MasterID即主机ID，主机ID非常不建议为0x00，建议设置为CANID基础上＋0x10，例如下面的0x11.**

```c++
auto dm =damiao::Motor_Control(serial);
damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4310,0x05, 0x15);
```

## 3.使能电机

​	使能电机之前需要先进行添加电机操作

```c++
dm.addMotor(&M1);
dm.addMotor(&M2);
```

​	建议使能电机放在初始化的最后，在使能电机前面进行参数修改保存参数等操作。

```c++
dm.enable(M1);
dm.enable(M2);
```

​	此代码为兼容旧固件，关于旧版本电机固件，使能对应不同模式需要加上使能的模式（即需要使能电机对应的模式，并不能修改电机此时的模式）**注意需要使能电机此时对应的模式，并不能修改电机内部的模式**

```c++
dm.enable_old(Motor1,damiao::MIT_MODE);
dm.enable_old(Motor2,damiao::POS_VEL_MODE);
dm.enable_old(Motor3,damiao::VEL_MODE);
```

## 4.电机控制

​	目前驱动库支持4种控制模式。

**推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。**

### 4.1设置电机零点

​	将电机在失能状态下摆到需要设置为0点的位置，然后运行下面两行，电机将会将当前位置作为电机0点。

```c++
dm.set_zero_position(M1);
dm.set_zero_position(M2);
```

### 4.2 MIT控制模式

​	默认的control是mit控制模式，第一个参数电机对象，第二个是kp，第三个是kd，第四个是位置，第五个是速度，第六个是扭矩。具体请参考达妙手册进行控制，参数介绍在库中有详细描述

```c++
dm.control_mit(M1, 50, 0.3, 0, 0, 0);
```

### 4.3 位置速度模式

​	第一个参数是电机对象，第二个参数是对应的位置，第三个参数是旋转到该位置用的速度.具体的参数介绍已经写了函数文档。

```c++
float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
dm.control_pos_vel(M2, q*10, 5);
```

### 4.4速度模式

​	第二参数就是电机的转速。

```c++
dm.control_vel(M1, q*10);
```

### 4.5力位混合模式

​	第一个是电机对象，第二个是电机位置，第三个是电机速度范围是0-10000，第四个是电机电流范围为0-10000。具体详细请查看达妙文档。

```c++
dm.control_pos_force(M1,5,500, 1000);
```

## 5.电机内部参数更改

​	达妙电机新固件支持使用can进行电机模式修改，以及修改其他参数等操作。要求版本号5013及以上。具体请咨询达妙客服。**请注意所有保存参数、修改参数。请在失能模式下修改！！**

### 5.1电机控制模式更改

MIT_MODE ,POS_VEL_MODE,VEL_MODE,POS_FORCE_MODE

​	下面是修改的demo。并且代码会有返回值，如果是True那么说明设置成功了，如果不是也不一定没修改成功hhhh。**请注意这里模式修改只是当前有效，掉电后这个模式还是修改前的**

```c++
if(dm.switchControlMode(M1, damiao::MIT_MODE))
   std::cout << "Switch to MIT Success" << std::endl;
if(dm.switchControlMode(M2, damiao::POS_VEL_MODE))
   std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
```

**如果要保持电机控制模式，需要最后保存参数**

### 5.2 保存参数

​	默认电机修改模式等操作后参数不会保存到flash中，需要使用命令如下进行保存至电机的flash中。这一个例子如下。**请注意这一个代码就把所有修改的都保存到Motor1的flash中，并且请在失能模式下进行修改**，该函数内部有自动失能的代码，防止电机在使能模式下无法保存参数。函数内部延迟了一段时间等待电机参数修改完成

```c++
dm.save_motor_param(M1);
dm.save_motor_param(M2);
```

### 5.3 读取内部寄存器参数

​	内部寄存器有很多参数都是可以通过can线读取，具体参数列表请看达妙的手册。其中可以读的参数都已经在DM_Reg这个枚举类里面了。可以通过read_motor_param进行读取

```c++
std::cout<<"motor1 PMAX:"<<dm.read_motor_param(M1, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
std::cout<<"motor2 PMAX:"<<dm.read_motor_param(M2, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
```

### 5.4 改写内部寄存器参数

​	内部寄存器有一部分是支持修改的，一部分是只读的（无法修改）。通过调用change_motor_param这个函数可以进行寄存器内部值修改(例如damiao::UV_Value)。并且也如同上面读寄存器的操作一样，他的寄存器的值也会同步到电机对象的内部值。

```c++
if(dm.change_motor_param(M1, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout<<"motor1 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
if(dm.change_motor_param(M2, damiao::UV_Value, 12.6f))
    std::cout << "Change UV_Value Success" << std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
if(dm.change_motor_param(M1,damiao::CTRL_MODE,1))
    std::cout << "Change CTRL_MODE Success" << std::endl;
std::cout<<"motor1 CTRL_MODE:"<<dm.read_motor_param(M1, damiao::CTRL_MODE)<<std::endl;
```

