# 达妙电机Linux-C++驱动库
该驱动库对达妙客户提供的驱动库进行了魔改，增加了诸多功能。最低支持的C++版本为C++11

该驱动库目前只能在**Linux**下使用，特别适合使用ROS的宝宝

**欢迎加入QQ群：677900232 进行达妙电机技术交流。欢迎进入达妙店铺进行点击选购**[首页-达妙智能控制企业店-淘宝网 (taobao.com)](https://shop290016675.taobao.com/?spm=pc_detail.29232929/evo365560b447259.shop_block.dshopinfo.59f47dd6w4Z4dX)

### 1.引用达妙库

​	通过头文件引入达妙库

```c++
#include "damiao.h"
```

### 2.定义控制对象

​	对电机进行控制需要定义相关的对象。

​	首先是串口对象，达妙的USB转串口默认是921600波特率，然后串口号的选择大家根据自己linux设备下的串口号进行选择。

```c++
auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
```

​	然后要定义相关的电机控制对象和电机对象。其中电机对象中第一个参数是电机类型需要进行选择，第二个参数是电机的SlaveID即CANID即电机控制的ID，**第三个参数是MasterID即主机ID，主机ID不要设为0x00，建议设置为CANID基础上＋0x10，例如下面的0x11.**

**MasterID和SlaveID需要在达妙上位机进行设置！！如果出现问题请先检查MasterID是否不和SlaveID冲突，并且不为0x00**

**MasterID不要设置为0x00**

**MasterID不要设置为0x00**

**MasterID不要设置为0x00**

```c++
auto dm =damiao::Motor_Control(serial);
damiao::Motor M1(damiao::DM4310,0x01, 0x11);
damiao::Motor M2(damiao::DM4310,0x05, 0x15);
```

### 3.电机状态

#### 3.1 添加电机

添加电机是addMotor。

```c++
dm.addMotor(&M1);
dm.addMotor(&M2);
```

#### 3.2使能电机

**建议：如果要修改电机参数。建议使能放在最后**

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

#### 3.3设置电机零点

将电机在失能状态下摆到需要设置为0点的位置，然后运行下面两行，电机将会将当前位置作为电机0点。

```c++
dm.set_zero_position(M1);
dm.set_zero_position(M2);
```

#### 3.4 失能电机

```c++
dm.disable(M1);
dm.disable(M2);
```

#### 3.5 电机状态获取

达妙电机默认是需要每发送一帧控制指令才能获得当前电机力矩、位置、速度等信息。如果在没有发送控制指令的过程中想要获得电机此时的状态可以通过以下指令。

```c++
dm.refresh_motor_status(M1);
dm.refresh_motor_status(M2);
std::cout<<"motor1--- POS:"<<M1.Get_Position()<<" VEL:"<<M1.Get_Velocity()<<" CUR:"<<M1.Get_tau()<<std::endl;
std::cout<<"motor2--- POS:"<<M2.Get_Position()<<" VEL:"<<M2.Get_Velocity()<<" CUR:"<<M2.Get_tau()<<std::endl;
```

通过**refresh_motor_status**这个函数可以获得当前电机的状态，并保存到对应的电机。

### 4.电机控制模式

**推荐在每帧控制完后延迟2ms或者1ms，usb转can默认有缓冲器没有延迟也可使用，但是推荐加上延迟。**

#### 4.1MIT模式

​	默认的control是mit控制模式，第一个参数电机对象，第二个是kp，第三个是kd，第四个是位置，第五个是速度，第六个是扭矩。具体请参考达妙手册进行控制，参数介绍在库中有详细描述

```c++
dm.control_mit(M1, 50, 0.3, 0, 0, 0);
```

#### 4.2位置速度模式

​	第一个参数是电机对象，第二个参数是对应的位置，第三个参数是旋转到该位置用的速度.具体的参数介绍已经写了函数文档。

```c++
float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
dm.control_pos_vel(M2, q*10, 5);
```

#### 4.3 速度模式

​	第二参数就是电机的转速。

```c++
dm.control_vel(M1, q*10);
```

#### 4.4力位混合模式

​	第一个是电机对象，第二个是电机位置，第三个是电机速度范围是0-10000，第四个是电机电流范围为0-10000。具体详细请查看达妙文档。

```c++
dm.control_pos_force(M1,5,500, 1000);
```

### 5.电机状态读取

电机的各个状态都保存在对应的电机对象中，需要调用可以用如下几个函数。

**请注意！达妙的电机状态是每次发了控制帧或者刷新状态(refresh_motor_status 函数)后才能刷新电机对象的当前各个信息。！！**

**达妙电机是一发一收模式，只有发送指令电机才会返回当前状态，电机才会更新**

```c++
float pos = M1.Get_Position();  //获得电机位置
float vel = M1.Get_Velocity();  //获得电机速度
float tau = M1.Get_tau();       //获得电机力矩
```

### 6.电机内部参数更改

达妙电机新固件支持使用can进行电机模式修改，以及修改其他参数等操作。要求版本号5013及以上。具体请咨询达妙客服。**请注意所有保存参数、修改参数。请在失能模式下修改！！**

#### 6.1电机控制模式更改

通过下面的函数可以对电机的控制模式进行修改。支持MIT,POS_VEL,VEL,Torque_Pos。四种控制模式在线修改。下面是修改的demo。并且代码会有返回值，如果是True那么说明设置成功了，如果不是也不一定没修改成功hhhh。**请注意这里模式修改只是当前有效，掉电后这个模式还是修改前的**

```c++
if(dm.switchControlMode(M1, damiao::MIT_MODE))
   std::cout << "Switch to MIT Success" << std::endl;
if(dm.switchControlMode(M2, damiao::POS_VEL_MODE))
   std::cout << "Switch to POS_VEL_MODE Success" << std::endl;
```

**如果要保持电机控制模式，需要最后保存参数**

#### 6.2保存参数

​	默认电机修改模式等操作后参数不会保存到flash中，需要使用命令如下进行保存至电机的flash中。这一个例子如下。**请注意这一个代码就把所有修改的都保存到Motor1的flash中，并且请在失能模式下进行修改**，该函数内部有自动失能的代码，防止电机在使能模式下无法保存参数。函数内部延迟了一段时间等待电机参数修改完成

```c++
dm.save_motor_param(M1);
dm.save_motor_param(M2);
```

#### 6.3 读取内部寄存器参数

​	内部寄存器有很多参数都是可以通过can线读取，具体参数列表请看达妙的手册。其中可以读的参数都已经在DM_Reg这个枚举类里面了。可以通过read_motor_param进行读取

```c++
std::cout<<"motor1 PMAX:"<<dm.read_motor_param(M1, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M1, damiao::UV_Value)<<std::endl;
std::cout<<"motor2 PMAX:"<<dm.read_motor_param(M2, damiao::PMAX)<<std::endl;
std::cout<<"motor2 UV_Value:"<<dm.read_motor_param(M2, damiao::UV_Value)<<std::endl;
```

#### 6.4改写内部寄存器参数

​	内部寄存器有一部分是支持修改的，一部分是只读的（无法修改）。通过调用change_motor_param这个函数可以进行寄存器内部值修改(例如damiao::UV_Value)。并且也如同上面读寄存器的操作一样，他的寄存器的值也会同步到电机对象的内部值。

**请注意这个修改内部寄存器参数，掉电后会恢复为修改前的，并没有保存**

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

