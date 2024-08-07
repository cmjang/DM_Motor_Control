#include "damiao.h"
#include "unistd.h"
#include <cmath>

int main(int argc , char** argv)
{
  auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
  auto dm =damiao::Motor_Control(serial);
  damiao::Motor M1(damiao::DM4310,0x01, 0x11);
  damiao::Motor M2(damiao::DM4310,0x05, 0x15);
  dm.addMotor(&M1);
  dm.addMotor(&M2);
  dm.enable(M1);
  dm.enable(M2);
  dm.switchControlMode(M1, damiao::VEL_MODE);
  dm.switchControlMode(M2, damiao::POS_VEL_MODE);
  dm.save_motor_param(M1);
  dm.save_motor_param(M2);
  // dm.zero_position(M1);
  sleep(2);
  
  while (true)
  {
    float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
    // dm.control(M1, 50, 0.3, 0, 0, 0);
    // dm.control_pos_vel(M1,q*10,5);
    dm.control_vel(M1, q*10);
    dm.control_pos_vel(M2, q*10, 5);
    std::cout << "m1: " << M1.Get_Position() << " " << M1.Get_Velocity() << " " << M1.Get_tau()  << std::endl;

  }

  return 0;
}
