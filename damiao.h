#ifndef DAMIAO_H
#define DAMIAO_H

#include "SerialPort.h"
#include <utility>
#include <vector>
#include <unordered_map>
#include <array>
#define MILESTRO_DECLARE_NON_COPYABLE(className)                                                                         \
    className(const className&) = delete;                                                                              \
    className& operator=(const className&) = delete;

#define POS_MODE 0x100
#define SPEED_MODE 0x200
#define POSI_MODE 0x300
namespace damiao
{

#pragma pack(1)
#define Motor_id uint32_t
    enum DM_Motor_Type
    {
        DM4310,
        DM4310_48V,
        DM4340,
        DM6006,
        DM8006,
        DM8009,
        Num_Of_Motor
    };

    enum Control_Mode
    {
        MIT_MODE=1,
        POS_VEL_MODE=2,
        VEL_MODE=3,
        POS_FORCE_MOD0E=4,
    };

    typedef struct
    {
        uint8_t freamHeader;
        uint8_t CMD;// 命令 0x00: 心跳
        //     0x01: receive fail 0x11: receive success
        //     0x02: send fail 0x12: send success
        //     0x03: set baudrate fail 0x13: set baudrate success
        //     0xEE: communication error 此时格式段为错误码
        //     8: 超压 9: 欠压 A: 过流 B: MOS过温 C: 电机线圈过温 D: 通讯丢失 E: 过载
        uint8_t canDataLen: 6; // 数据长度
        uint8_t canIde: 1; // 0: 标准帧 1: 扩展帧
        uint8_t canRtr: 1; // 0: 数据帧 1: 远程帧
        uint32_t CANID; // 电机反馈的ID
        uint8_t canData[8];
        uint8_t freamEnd; // 帧尾
    } CAN_Recv_Fream;

    typedef struct
    {
        uint8_t freamHeader[2] = {0x55, 0xAA}; // 帧头
        uint8_t freamLen = 0x1e; // 帧长
        uint8_t CMD = 0x01; // 命令 1：转发CAN数据帧 2：PC与设备握手，设备反馈OK 3: 非反馈CAN转发，不反馈发送状态
        uint32_t sendTimes = 1; // 发送次数
        uint32_t timeInterval = 10; // 时间间隔
        uint8_t IDType = 0; // ID类型 0：标准帧 1：扩展帧
        uint32_t CANID=0x01; // CAN ID 使用电机ID作为CAN ID
        uint8_t frameType = 0; // 帧类型 0： 数据帧 1：远程帧
        uint8_t len = 0x08; // len
        uint8_t idAcc=0;
        uint8_t dataAcc=0;
        uint8_t data[8]={0};
        uint8_t crc=0; // 未解析，任意值

        void modify(const Motor_id id, const uint8_t* send_data)
        {
            CANID = id;
            std::copy(send_data, send_data+8, data);
        }
    } CAN_Send_Fream;

#pragma pack()

    typedef struct
    {
        float Q_MIN;
        float Q_MAX;
        float DQ_MAX;
        float TAU_MAX;
    }Limit_param;

    class Motor
    {
    private:
        /* data */
        Motor_id Master_id;
        Motor_id Slave_id;
        float kp=0;
        float kd=0;
        float cmd_q=0;
        float cmd_dq=0;
        float cmd_tau=0;
        float state_q=0;
        float state_dq=0;
        float state_tau=0;
        Limit_param limit_param;
        DM_Motor_Type Motor_Type;

      

    public:
        Motor(DM_Motor_Type Motor_Type,Motor_id Slave_id,Motor_id Master_id)
        {
            Limit_param limit_param[Num_Of_Motor]=
                    {
                            { -12.5, 12.5, 30, 10 }, // DM4310
                            { -12.5, 12.5, 30, 10 }, // DM4340
                            { -12.5, 12.5, 30, 10 }, // DM6006
                            { -12.5, 12.5, 30, 10 }  // DM8009
                    };
            this->limit_param = limit_param[Motor_Type];
            this->Motor_Type = Motor_Type;
            this->Master_id = Master_id;
            this->Slave_id = Slave_id;
        }

        Motor(){
            this->Master_id = 0x01;
            this->Slave_id = 0x00;
            Limit_param limit_param[Num_Of_Motor]=
            {
                            { -12.5, 12.5, 30, 10 }, // DM4310
                            { -12.5, 12.5, 30, 10 }, // DM4340
                            { -12.5, 12.5, 30, 10 }, // DM6006
                            { -12.5, 12.5, 30, 10 }  // DM8009
            };
            this->limit_param = limit_param[DM4310];
        }
        ~Motor();
         void recv_data(float q, float dq, float tau)
        {
            this->state_q = q;
            this->state_dq = dq;
            this->state_tau = tau;
        }

        void save_cmd(float cmd_kp, float cmd_kd, float q, float dq, float tau)
        {
            this->kp = cmd_kp;
            this->kd = cmd_kd;
            this->cmd_q = q;
            this->cmd_dq = dq;
            this->cmd_tau = tau;
        }
        DM_Motor_Type GetMotorType() const
        {
            return this->Motor_Type;
        }

        Motor_id GetMasterId() const
        {
            return this->Master_id;
        }

        Motor_id GetSlaveId() const
        {
            return this->Slave_id;
        }
        float Get_Position() const
        {
            return this->state_q;
        }
        float Get_Velocity() const
        {
            return this->state_dq;
        }
        float Get_tau() const
        {
            return this->state_tau;
        }
        Limit_param get_limit_param()
        {
            return limit_param;
        }
    };

    Motor::~Motor()
    {
    }

/**
 * @brief 达妙科技  电机控制
 *
 * 使用USB转CAN进行通信，linux做虚拟串口
 */
    class Motor_Control
    {
    public:

        /*
        * @brief 构造函数
        * @param serial 串口对象
        */
        Motor_Control(SerialPort::SharedPtr serial = nullptr): serial_(std::move(serial))
        {
            if (serial_ == nullptr) {
                //default serial port
                serial_ = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
            }
        }
        ~Motor_Control()
        {
        }

        /*
        * @brief 使能电机
        */
        void enable(const Motor& damiao)
        {
            control_cmd(damiao.GetSlaveId(), 0xFC);
        }
        /*
        * @brief 关闭电机
        * @param 电机对象
        */
        void disable(const Motor& damiao) {
            control_cmd(damiao.GetSlaveId(), 0xFD);
        }

        /*
        * @brief 保存位置零点
        * @param 电机对象
        */
        void zero_position(const Motor& damiao)
        {
            control_cmd(damiao.GetSlaveId(), 0xFE);
        }

        /* @description: 控制电机
          *@param q: 位置
          *@param dq: 速度
          *@param tau: 扭矩
          *@param kp: 比例系数
          *@param kd: 微分系数
        */
        void control(Motor &DM_Motor, float kp, float kd, float q, float dq, float tau)
        {
            // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
            static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
                float span = xmax - xmin;
                float data_norm = (x - xmin) / span;
                uint16_t data_uint = data_norm * ((1 << bits) - 1);
                return data_uint;
            };
            Motor_id id = DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto& m = motors[id];
            m->save_cmd(kp, kd, q, dq, tau);
            uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
            uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
            Limit_param limit_param_cmd = m->get_limit_param();
            uint16_t q_uint = float_to_uint(q, limit_param_cmd.Q_MIN, limit_param_cmd.Q_MAX, 16);
            uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX,limit_param_cmd.DQ_MAX, 12);
            uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = (q_uint >> 8) & 0xff;
            data_buf[1] = q_uint & 0xff;
            data_buf[2] = dq_uint >> 4;
            data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
            data_buf[4] = kp_uint & 0xff;
            data_buf[5] = kd_uint >> 4;
            data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
            data_buf[7] = tau_uint & 0xff;

            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
            this->recv();
        }

        void control_pos_vel(Motor &DM_Motor,float pos,float vel)
        {
            Motor_id id = DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto& m = motors[id];
            uint8_t *pbuf, *vbuf;
            pbuf=(uint8_t*)&pos;
            vbuf=(uint8_t*)&vel;
            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = pbuf[0];
            data_buf[1] = pbuf[1];
            data_buf[2] = pbuf[2];
            data_buf[3] = pbuf[3];
            data_buf[4] = vbuf[0];
            data_buf[5] = vbuf[1];
            data_buf[6] = vbuf[2];
            data_buf[7] = vbuf[3];
            id=id+POS_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
            this->recv();
        }

        void control_vel(Motor &DM_Motor,float vel)
        {
            Motor_id id =DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto& m = motors[id];
            uint8_t  *vbuf;
            vbuf=(uint8_t*)&vel;
            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = vbuf[0];
            data_buf[1] = vbuf[1];
            data_buf[2] = vbuf[2];
            data_buf[3] = vbuf[3];
            data_buf[4] = 0;
            data_buf[5] = 0;
            data_buf[6] = 0;
            data_buf[7] = 0;
            id=id+SPEED_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
            this->recv();
        }

        void control_pos_force(Motor &DM_Motor,float pos, uint16_t vel, uint16_t i)
        {
            Motor_id id =DM_Motor.GetSlaveId();
            if(motors.find(id) == motors.end())
            {
                throw std::runtime_error("Motor_Control id not found");
            }
            auto& m = motors[id];
            uint8_t *pbuf, *vbuf, *ibuf;
            pbuf=(uint8_t*)&pos;
            vbuf=(uint8_t*)&vel;
            ibuf=(uint8_t*)&i;
            std::array<uint8_t, 8> data_buf{};
            data_buf[0] = pbuf[0];
            data_buf[1] = pbuf[1];
            data_buf[2] = pbuf[2];
            data_buf[3] = pbuf[3];
            data_buf[4] = vbuf[0];
            data_buf[5] = vbuf[1];
            data_buf[6] = ibuf[0];
            data_buf[7] = ibuf[1];
            id=id+POSI_MODE;
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
            this->recv();
        }

        void recv()
        {
            serial_->recv((uint8_t*)&recv_data, 0xAA, sizeof(CAN_Recv_Fream));

            if(recv_data.CMD == 0x11 && recv_data.freamEnd == 0x55) // receive success
            {
                static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
                    float span = xmax - xmin;
                    float data_norm = float(x) / ((1 << bits) - 1);
                    float data = data_norm * span + xmin;
                    return data;
                };

                auto & data = recv_data.canData;

                uint16_t q_uint = (uint16_t(data[1]) << 8) | data[2];
                uint16_t dq_uint = (uint16_t(data[3]) << 4) | (data[4] >> 4);
                uint16_t tau_uint = (uint16_t(data[4] & 0xf) << 8) | data[5];

                if(motors.find(recv_data.CANID) == motors.end())
                {
                    std::cout << "Unknown motor id: " << std::hex << recv_data.CANID << std::endl;
                    return;
                }

                auto m = motors[recv_data.CANID];
                Limit_param limit_param_recv = m->get_limit_param();
                float recv_q = uint_to_float(q_uint, limit_param_recv.Q_MIN, limit_param_recv.Q_MAX, 16);
                float recv_dq = uint_to_float(dq_uint, -limit_param_recv.DQ_MAX, limit_param_recv.DQ_MAX, 12);
                float recv_tau = uint_to_float(tau_uint, -limit_param_recv.TAU_MAX, limit_param_recv.TAU_MAX, 12);
                m->recv_data(recv_q, recv_dq, recv_tau);
                return;
            }
            else if (recv_data.CMD == 0x01) // receive fail
            {
                /* code */
            }
            else if (recv_data.CMD == 0x02) // send fail
            {
                /* code */
            }
            else if (recv_data.CMD == 0x03) // send success
            {
                /* code */
            }
            else if (recv_data.CMD == 0xEE) // communication error
            {
                /* code */
            }
        }
        /**
         * @brief 添加电机
         * @param DM_Motor 电机对象地址
         * 
         */
        void addMotor(Motor *DM_Motor)
        {
            motors.insert({DM_Motor->GetSlaveId(), DM_Motor});
            if (DM_Motor->GetMasterId() != 0)
            {
                motors.insert({DM_Motor->GetMasterId(), DM_Motor});
            }
        }

        void read_motor_param(Motor &DM_Motor,uint8_t RID)
        {
            uint8_t id = DM_Motor.GetSlaveId();
            std::array<uint8_t, 8> data_buf{id, 0x00, 0x33, RID, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
        }

        void write_motor_param(Motor &DM_Motor,uint8_t RID,const uint8_t data[4])
        {
            uint8_t id = DM_Motor.GetSlaveId();
            std::array<uint8_t, 8> data_buf{id, 0x00, 0x55, RID, 0x00, 0x00, 0x00, 0x00};
            data_buf[4] = data[0];
            data_buf[5] = data[1];
            data_buf[6] = data[2];
            data_buf[7] = data[3];
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
        }

        void switchControlMode(Motor &DM_Motor,Control_Mode mode)
        {
            uint8_t write_data[4]={(uint8_t)mode, 0x00, 0x00, 0x00};
            write_motor_param(DM_Motor,10,write_data);
        }

        void save_motor_param(Motor &DM_Motor)
        {
            uint8_t id = DM_Motor.GetSlaveId();
            std::array<uint8_t, 8> data_buf{id, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
            send_data.modify(0x7FF, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
        }

    private:
        void control_cmd(Motor_id id , uint8_t cmd)
        {
            std::array<uint8_t, 8> data_buf = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd};
            send_data.modify(id, data_buf.data());
            serial_->send((uint8_t*)&send_data, sizeof(CAN_Send_Fream));
            usleep(1000);
            recv();
        }
        std::unordered_map<Motor_id, Motor*> motors;
        SerialPort::SharedPtr serial_;  //serial port
        CAN_Send_Fream send_data; //send data frame
        CAN_Recv_Fream recv_data;//receive data frame
    };

}; 

#endif