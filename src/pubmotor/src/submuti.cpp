#include "ros/ros.h"
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <serial/serial.h>
#include<geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <boost/thread.hpp>
#include "pubmotor/motor.h"
#include <chrono>
#include <cmath>
#include <thread>
#include <signal.h>

#define MOTOR_NUM 6

#define sBUFFERSIZE 100 // send buffer size串口发送缓存长度
#define rBUFFERSIZE 100 // receive buffer size 串口接收缓存长度
#define CLOSED 0
#define OPENED 1
#define CLOSE_ARRIVALED 2
#define OPEN_ARRIVALED 1
#define MOVING 0
#define WHEEL_DISTANCE 11
#define LEFT 1
#define RIGHT 2
#define WHEEL_GROUP_DIS 590 // the distance of steer wheel groups
#define H WHEEL_GROUP_DIS/2
#define WHEEL_WITHIN_DIS 282  
#define W WHEEL_WITHIN_DIS/2
#define FRONT_REAR_DIS 2500  // the distance of motor modules between rear and front
#define L FRONT_REAR_DIS
#define PI 3.14159
#define STATIC_YAW 5.0/180*M_PI
#define STATIC_R 2000

using namespace std;
unsigned char s_buffer[sBUFFERSIZE]; //发送缓存
unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

vector<unsigned char>  ser_shell;

float wheel_angle1 = 0;
float wheel_angle2 = 0;

uint8_t Left_Is_Arrival = 0;
uint8_t Right_Is_Arrival = 0;
uint8_t Raise_Car = 0;

unsigned char save_data[8] = {0,0,0,0, 0, 0, 0, 0};

typedef struct PID_Date
{
	float error ;
	float last_error ;
	float earlier_error ;
	float K_P ;
	float K_I ;
	float K_D ;
	float I_Separate ;
	float I_Limit ;
	float Out_MAX ;
	float Out_MIN ; 
	float Dead_Size;
   double Output ;
}PID_Date;
PID_Date YAW_PID ;

struct Wheel_Speed_Info{
   double yaw_error_speed;
   float yaw_turning_speed;
   float liner_speed;
   double target_angle;
};
Wheel_Speed_Info Wheel_Speed[4];

struct ROBOT_SPEED{
   float vel_x;
   float vel_yaw;
   float adjust_yaw;
};
ROBOT_SPEED ROBOT_speed = {0,0};
float joy_vel_x, joy_vel_yaw,syn_vel_x, syn_vel_yaw,syn_circle_R = 0.0f;
float turning_yaw = 5.0/180*M_PI;
float turning_radius = 2000;

serial::Serial ser2; //init the port Encoder

unsigned int Crc_Count(unsigned char pbuf[],unsigned char num)
{
   int i,j; unsigned int wcrc=0xffff;
   for(i=0;i<num;i++)
   {
      wcrc^=(unsigned int)(pbuf[i]);
      for (j=0;j<8;j++)
      {
         if(wcrc&0x0001)
         {
         wcrc>>=1; wcrc^=0xa001;
         }
         else
         wcrc>>=1;
      }
   }
   return wcrc;
}

void PID_Parameter_Speed_Init(PID_Date * PID,float Pi,float Ki,float Di,float Out_MAX,float Dead_Size,float I_Separate , float I_Limit  )
{
  PID->K_P = Pi;
	PID->K_I = Ki;
	PID->K_D = Di;
	PID->Dead_Size = Dead_Size;
	PID->Out_MAX   = Out_MAX ;
	PID->Out_MIN   = -Out_MAX;
	PID->I_Limit = I_Limit ;
	PID->I_Separate = I_Separate ;
	
	PID->error = PID->last_error = PID->earlier_error =0;
	PID->Output = 0;
}

double PID_Position_Calculate(PID_Date * PID,float expect,float Encoder_Count)
{
	static float integral = 0 ; 
	       float differential = 0;
  PID->error   = expect     - Encoder_Count;                  
	differential = PID->error - PID->last_error ;        
	
	if( PID->I_Separate == 0 )
		 integral += PID->error  ;
	else
	{

	if( abs(PID->error) < PID->I_Separate  )              
		 integral += PID->error  ;  
  else 	
		 integral = 0 ;
  }

	if( integral >  PID->I_Limit ) integral =  PID->I_Limit ;
	if( integral < -PID->I_Limit ) integral = -PID->I_Limit ;
	
	if( abs(PID->error) > PID->Dead_Size  )
	{	
	 PID->Output = PID->K_P * PID->error +PID->K_I  * integral + PID->K_D  * differential ;
	}
	else	PID->Output = 0;

	PID->last_error = PID->error ;	
	
	if( PID->Output > PID->Out_MAX )   PID->Output = PID->Out_MAX ;
	if( PID->Output < PID->Out_MIN )   PID->Output = PID->Out_MIN ;
  			
	return PID->Output ;	
}

uint32_t combineBytes(const unsigned char* bytes, int length) {
    uint32_t result = 0;
    for (int i = 0; i < length; ++i) {
        result = (result << 8) | static_cast<uint32_t>(bytes[i]);
    }
    return result;
}

/*Solve the decoder data to angle*/
uint8_t Decoder_wheel_Angle(void){
   
   uint8_t ID = r_buffer[0];
   int decimalValue = 0;
   uint8_t Len = static_cast<int>(r_buffer[2]);
   // cout<<Len<<":"<<r_buffer[2]<<endl;
   // Check if there are enough bytes in the buffer
    if (sizeof(r_buffer) / sizeof(r_buffer[0]) < 3 + Len) {
        std::cerr << "Insufficient data" << std::endl;
        return 0;
    }
   if(ID == 0x02){
      uint32_t decimalValue = combineBytes(r_buffer + 3, Len);
      wheel_angle1 = decimalValue*360.0/1024/7-180.0;
      ROS_INFO("NO.1 steering wheel the angle is :%f", wheel_angle1);
      return 1;
   }else if(ID == 0x01){
      uint32_t decimalValue = combineBytes(r_buffer + 3, Len);
      wheel_angle2 = decimalValue*360.0/1024/7 - 180.0;
      // wheel_angle2 -= 180.0;
      ROS_INFO("NO.2 steering wheel the angle is :%f", wheel_angle2);
      return 1;
   }
   return 0;
   
}

uint8_t Request_encoder_data(uint8_t index){
   unsigned char request_data1[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
   unsigned char request_data2[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x38};
   switch(index){
      case 1:{
         ser2.write(request_data1,8);
         // ROS_INFO_STREAM("Sending request code 1, waiting for response");
         while(1){
            if(ser2.available()){
               // ROS_INFO_STREAM("executing the data1");
               size_t bytes_read = ser2.read(r_buffer, ser2.available());
               // std::cout << "Bytes read: " << bytes_read << std::endl;
               // std::vector<unsigned char> hex_data(r_buffer, r_buffer + bytes_read);
               // for (size_t i = 0; i < bytes_read; i++) {
               //       printf("%02X ", hex_data[i]);
               // }
               // printf("\r\n");
               uint8_t rec =  Decoder_wheel_Angle();
               if(rec ==1){
                  return 2; // double steering wheel
               }else{
                  return 1;
               }
            }
         }
      }break;
      case 2:{
         ser2.write(request_data2,8);
         // ROS_INFO_STREAM("Sending request code 2, waiting for response");
         while(1){
            if(ser2.available()){
               // ROS_INFO_STREAM("executing the data2");
               size_t result  = ser2.read(r_buffer,ser2.available());
               // cout<< result<<endl;
               // for (int i = 0; i < result; i++)
               // {
               //    printf("%02X ", r_buffer[i]);
               // }
               // printf("\r\n");
               uint8_t rec =  Decoder_wheel_Angle();
               if(rec ==1){
                  return 1;
               }else{
                  return 2;
               }
            }
         }
      }break;
      default:break;
   }
}

float Wheel_encoder_Calculate(float yaw_angle_error){
   float wheel_speed = yaw_angle_error*W;
   return wheel_speed;
}

/*parament: turning type left or right,turning circle's radius, turning circle's yaw speed*/
void Turning_Wheel_Cal(int turning_type, float circle_R, float turnings_yaw){
   double left_steer_angle, right_steer_angle = 0;
   float left_circle_R,right_circle_R = 0;

   if(turning_type == 1){
      /*calculate the two group steering wheel's angle*/
      left_steer_angle = -(PI-atan2(L, -(circle_R - H)))*180.0f/PI;
      right_steer_angle = -(PI-atan2(L, -(circle_R +H)))*180.0f/PI;
      // left_steer_angle = -90.0;
      // right_steer_angle = -90.0;
      // Wheel_Speed[0].target_angle = left_steer_angle;
      // Wheel_Speed[1].target_angle = right_steer_angle;
      Wheel_Speed[0].target_angle = right_steer_angle;
      Wheel_Speed[1].target_angle = left_steer_angle;
      /*calculate  the two group steering wheel's turning circle radius*/
      left_circle_R = sqrt((circle_R - H)*(circle_R - H) + L*L);
      right_circle_R = sqrt((circle_R + H)*(circle_R + H) + L*L);
      Wheel_Speed[0].liner_speed = (left_circle_R)*turnings_yaw/1000;
      Wheel_Speed[1].liner_speed = (right_circle_R)*turnings_yaw/1000;
   }else if(turning_type == 2){
      left_steer_angle = atan2(L, (circle_R + H))*180.0f/PI;
      right_steer_angle = atan2(L, (circle_R - H))*180.0f/PI;
      // left_steer_angle = 90.0;
      // right_steer_angle = 90.0;
      // Wheel_Speed[0].target_angle = left_steer_angle;
      // Wheel_Speed[1].target_angle = right_steer_angle;
      Wheel_Speed[0].target_angle = right_steer_angle;
      Wheel_Speed[1].target_angle = left_steer_angle;
      left_circle_R = sqrt((circle_R + H)*(circle_R + H) + L*L);
      right_circle_R = sqrt((circle_R - H)*(circle_R - H) + L*L);
      Wheel_Speed[0].liner_speed = (left_circle_R )*turnings_yaw/1000;
      Wheel_Speed[1].liner_speed = (right_circle_R )*turnings_yaw/1000;
   }
   cout<<"the turning type is:"<<turning_type<<endl;
   cout<<"the NO.1 steering target angle is:"<<left_steer_angle<<" ;the NO.2 steering target angle is:"<<right_steer_angle<<endl;
   cout<<"the NO.1 steering left target speed is:"<<Wheel_Speed[0].liner_speed<<" ;the NO.2 steering right target speed is:"<<Wheel_Speed[1].liner_speed<<endl;
   // cout<<"the NO.2 steering left target speed is:"<<Wheel_Speed[2].liner_speed<<" ;the NO.2 steering right target speed is:"<<Wheel_Speed[3].liner_speed<<endl;
}

class multiThreadListener
{
public:
	multiThreadListener()
	{
    pub = n.advertise<pubmotor::motor>("motor_message",10);
    sub = n.subscribe("motor_message", 1, &multiThreadListener::chatterCallback1,this);
    can1 = n.subscribe("received_messages", 1, &multiThreadListener::chatterCallback3,this);
    sub_agv_vel = n.subscribe("rear_agv_vel", 1, &multiThreadListener::AgvVelotryCallback, this);
    synergy_vel = n.subscribe("cmd_vel", 1, &multiThreadListener::SynVelotryCallback, this);
   //  sub_agv_posiiton = n.subscribe("Odometry", 1, &multiThreadListener::AgvPositionCallback, this);
    pubcan = n.advertise<can_msgs::Frame>("sent_messages", 100);
	}
  void chatterCallback1(const pubmotor::motor::ConstPtr& msg);
  void chatterCallback3(const can_msgs::Frame::ConstPtr &msg);
  void AgvVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void SynVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);
  // void publish_data(string id, float velocity,float acceleration);
  void publish_can(vector<unsigned char>& data, uint8_t Len);
  void Adjust_Steering_WHEEL(void);

  void CAN_msg_classify(const unsigned char  *data, uint32_t motor_id, uint8_t Len);
  vector<unsigned char >send_data={0x12,0x34,0x00,0x00,0x00,0x00,0x00,0x56};
  void PDO_deal_info(const int slave_id,const int pdo_id, unsigned char *pdo_data);
  void Motor_Info(int motor_id);
  void NMT_Rec_Func(const int slave_id, unsigned char *pdo_data);
  void ERROR_CODE(const int slave_id, unsigned char *err_data);
  bool SDO_rec_FUNC(const int slave_id, const unsigned int mode, unsigned char *sdo_data, unsigned char *save_data);

  void PDO_Send_func(const int slave_id,const int pdo_id,const vector<unsigned char>& pdo_data,const unsigned int dlen);
  void NMT_ShellFunc(const int slave_id,unsigned char shell);

private:
   typedef struct{
      int Actual_Pos = 0;
      float Actual_Val = 0;
      int Status_word = 0;
      unsigned int Inputs = 0;
      unsigned int Status = 0;
      unsigned char Mode = 0;
   }Motor_INFO;
   Motor_INFO testMotor_info[MOTOR_NUM];
  int slave_id;
  unsigned int w_or_r = 0;/*SDO：1为写入数据，2为读取数据*/
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber can1;
  ros::Subscriber sub_agv_vel;
  ros::Subscriber synergy_vel;
  ros::Subscriber sub_agv_posiiton;
  ros::Publisher pub;
  ros::Publisher pubcan;
};

void multiThreadListener::Motor_Info(int motor_id){
  cout<<"Motor"<<motor_id<<" the speed is: "<<testMotor_info[motor_id-1].Actual_Val<<", Motor"<<motor_id<<" the position is: "<<testMotor_info[motor_id-1].Actual_Pos<<endl;
}

/*PDO发送函数*/
void multiThreadListener::PDO_Send_func(const int slave_id,const int pdo_id,const vector<unsigned char>& pdo_data,const unsigned int dlen){
   // snd_thread_arg_t *thread_arg = (snd_thread_arg_t *)param;
   can_msgs::Frame can_frame_msg; 
   int ret = 0;
   unsigned char data[8];
   memcpy(data, pdo_data.data(), 8);

   /*确定COB_ID*/
   switch (pdo_id)
   {
   case 1:
   {
      can_frame_msg.id = 0x200 + slave_id; 
      break;
   }
   case 2:
   {
      can_frame_msg.id  = 0x300 + slave_id; 
      break;
   }
   case 3:
   {
      can_frame_msg.id = 0x400 + slave_id; 
      break;
   }
   case 4:
   {
      can_frame_msg.id  = 0x500 + slave_id; 
      break;
   }
   default:
      break;
   }

   /*一些CAN的通信配置*/
   can_frame_msg.dlc = dlen;  // Length of data
   can_frame_msg.is_rtr = false;  // false-数据帧；ture-远程帧
   can_frame_msg.is_extended = false;  // fasle-标准帧；ture-扩展帧
   can_frame_msg.is_error = false;		//false-非错误帧；ture-错误帧

   for ( int i = 0; i <dlen; i++ ) 
   {
      can_frame_msg.data[i] = pdo_data[i];
      // printf("%02X  ", pdo_data[i]);
   }
   // ROS_INFO("is sending can message");
  pubcan.publish( can_frame_msg);

}

void multiThreadListener::NMT_ShellFunc(const int slave_id,unsigned char shell){
   can_msgs::Frame can_frame_msg; 

   can_frame_msg.id = 0x0;
   can_frame_msg.dlc = 2;  // Length of data
   can_frame_msg.is_rtr = false;  // false-数据帧；ture-远程帧
   can_frame_msg.is_extended = false;  // fasle-标准帧；ture-扩展帧
   can_frame_msg.is_error = false;		//false-非错误帧；ture-错误帧

   can_frame_msg.data[0] = shell;
   can_frame_msg.data[1] = slave_id;

   pubcan.publish( can_frame_msg);

}
/*以下为根CAN接受有关的函数*/
void multiThreadListener::PDO_deal_info(const int slave_id,const int pdo_id, unsigned char *pdo_data)
{
   int ret;
   // cout<<"receive can data is motor "<<slave_id<<"POD:"<<pdo_id<<"message:";
   // for(int i = 0;i<8;i++){
   //  printf("%02X  ",pdo_data[i]);
   // }
   // printf("\r\n");
   switch (pdo_id)
   {
      /*TPDO1 接受为状态字*/
      case 1:
      {
         testMotor_info[slave_id-1].Status_word = pdo_data[0]|(pdo_data[1]<<8);
         break;
      }
      /*TPDO2 接受为状态字、实际位置、控制模式*/
      case 2:
      {
         testMotor_info[slave_id-1].Status_word = pdo_data[0]|(pdo_data[1]<<8);
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[2]|(pdo_data[3]<<8)|(pdo_data[4]<<16)|(pdo_data[5]<<24);
         testMotor_info[slave_id-1].Mode = pdo_data[6];
         break;
      }
      /*TPDO3 接受为实际位置和速度*/
      case 3:
      {
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[0]|(pdo_data[1]<<8)|(pdo_data[2]<<16)|(pdo_data[3]<<24);
         testMotor_info[slave_id-1].Actual_Val = pdo_data[4]|(pdo_data[5]<<8)|(pdo_data[6]<<16)|(pdo_data[7]<<24);
      }
      /*TPDO1 接受为实际位置和输入信号*/
      case 4:
      {
         testMotor_info[slave_id-1].Actual_Pos = pdo_data[0]|(pdo_data[1]<<8)|(pdo_data[2]<<16)|(pdo_data[3]<<24);
         testMotor_info[slave_id-1].Inputs = pdo_data[4]|(pdo_data[5]<<8)|(pdo_data[6]<<16)|(pdo_data[7]<<24);
         break;
      }
      default:
      break;
   };
   
/*to check the active arm is arriving while opening*/
   if(testMotor_info[4].Actual_Pos >= 5450000){
      Left_Is_Arrival = OPEN_ARRIVALED;
   }else if(testMotor_info[4].Actual_Pos <= -10000){
      Left_Is_Arrival = CLOSE_ARRIVALED;
   }else{
      Left_Is_Arrival = MOVING;
   }
   if(testMotor_info[5].Actual_Pos >= 5450000){
      Right_Is_Arrival = OPEN_ARRIVALED;
   }else if(testMotor_info[5].Actual_Pos <= -30000){
      Right_Is_Arrival = CLOSE_ARRIVALED;
   }else{
      Right_Is_Arrival = MOVING;
   }
}
/*NMT报文处理，读取到的数据存入电机状态*/

void multiThreadListener::NMT_Rec_Func(const int slave_id, unsigned char *pdo_data){
   testMotor_info[slave_id-1].Status = pdo_data[0];
   // cout<<"Motor status is:"<<testMotor_info[slave_id-1].Status<<endl;
}
// /*如果为错误代码进行显示*/
void multiThreadListener::ERROR_CODE(const int slave_id, unsigned char *err_data){
   int errcode = 0;
   errcode = err_data[0] | (err_data[1]<<8);
   printf("the Drive :%04x is WRONGING, the ERROR CODE is: %04X!!!" ,slave_id, errcode);
}

bool multiThreadListener::SDO_rec_FUNC(const int slave_id, const unsigned int mode, unsigned char *sdo_data, unsigned char *save_data)
{
   unsigned int address = 0;
   unsigned int chlid_add = 0;
   unsigned int response = 0;
   int data = 0;
   unsigned int ret = 0;

   address = sdo_data[1]|(sdo_data[2]<<8);
   chlid_add = sdo_data[3];
   switch (mode)
      {
         /*当写入数据时，返回函数中如果为0x60的代表写入成功，如果为0x80代表写入失败*/
         case 1:{
            if(sdo_data[0] == 0x60){
               printf("Write data to %04x  %02x True!!\r\n",address , chlid_add);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x80){
               printf("Write data to %04x  %02x FALSE!!!\r\n",address , chlid_add);
               ret = false;
               goto end;
            }
            break;
         }
         case 2:{
            data = sdo_data[4]|(sdo_data[5]<<8)|(sdo_data[6]<<16)|(sdo_data[7]<<24);
            if(sdo_data[0] == 0x4f){
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               memcpy(save_data,sdo_data, 1);
               goto end;
            }else if(sdo_data[0] == 0x4b){
               memcpy(save_data,sdo_data, 2);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x47){
               memcpy(save_data,sdo_data, 3);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x43){
               memcpy(save_data,sdo_data,4);
               printf("Read data from %04x  %02x True, the data is :%08x!!\r\n",address , chlid_add, data);
               ret = true;
               goto end;
            }else if(sdo_data[0] == 0x80){
               printf("Read data from %04x  %02x FALSE!!!\r\n",address , chlid_add);
               ret =false;
               goto end;
            }
            break;
         }
         default:
         break;
      }
end:
   return ret;
}

unsigned char buff[8] = {0,0,0,0,0,0,0,0};
void multiThreadListener::CAN_msg_classify(const unsigned char  *data, uint32_t can_id, uint8_t Len){
  int COB_id = 0;
  memcpy(buff, data, Len);// 将读取到的数据存入bull中
  slave_id = can_id&0x7f;//解析相应的从站ID，根据ID进行不同的处理
  COB_id = can_id>>7;//解析出COB_ID，方便后续的处理
  COB_id = COB_id<<7;
  switch (COB_id)//根据不同的COB_ID，判断所属的报文类型
  {
    /*ERROR message*/
    case 0x080:
    {
        ERROR_CODE(slave_id, buff);
        break;
    }
    /*PDO1*/
    case 0x180:
    {
        PDO_deal_info(slave_id, 1, buff);
        break;
    }
    /*PDO2*/
    case 0x280:
    {
        PDO_deal_info(slave_id, 2, buff);
        break;
    }
    /*PDO3*/
    case 0x380:
    {
        PDO_deal_info(slave_id, 3, buff);
        break;
    }
    /*PDO4*/
    case 0x480:
    {
        PDO_deal_info(slave_id, 4, buff); 
        break;
    }
    /* SDO receive message*/
    case 0x580:
    {
        SDO_rec_FUNC(slave_id, w_or_r, buff, save_data);
        break;
    }
    /*NMT reflect drive message*/
    case 0x700:
    {
        NMT_Rec_Func(slave_id, buff);
        break;
    }
    default:
    break;
  }
}


// void multiThreadListener::publish_data(string id, float velocity,float acceleration){
//   pubmotor::motor motor_msg;
//   motor_msg.id = id;
//   motor_msg.velocity = velocity;
//   motor_msg.acceleration = acceleration;
//   for(int i=0;i++;i<10){pub.publish(motor_msg);};
//   }; 
void multiThreadListener:: publish_can( vector<unsigned char>& data, uint8_t Len)
{
   can_msgs::Frame can_frame_msg;
   can_frame_msg.dlc = 8;
   // 分开赋值
   can_frame_msg.data[0] = data[0];
   can_frame_msg.data[1] = data[1];
   can_frame_msg.data[2] = data[2];
   can_frame_msg.data[3] = data[3];
   can_frame_msg.data[4] = data[4];
   can_frame_msg.data[5] = data[5];
   can_frame_msg.data[6] = data[6];
   can_frame_msg.data[7] = data[7];
   // ROS_INFO("send can message");
   pubcan.publish( can_frame_msg);
  };

void multiThreadListener::Adjust_Steering_WHEEL(void){
   int motor1_speed,motor2_speed, motor3_speed, motor4_speed = 0;
   int slave_id1 = 1;
   int slave_id2 = 2;
   int slave_id3 = 3;
   int slave_id4 = 4;

   vector<unsigned char> shell_data1 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data2 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data3 = {0x0f,0,0,0, 0, 0, 0x03, 0};
   vector<unsigned char> shell_data4 = {0x0f,0,0,0, 0, 0, 0x03, 0};

   float vel_x, vel_yaw, vel_adjust_yaw, vel_pole, vel_e_pole, yaw_angle1, yaw_angle2, yaw_speed1, yaw_speed2 = 0;
   float target_angle1, target_angle2 = 0;

   if(joy_vel_x == 0 && syn_vel_x != 0){
      ROBOT_speed.vel_x = syn_vel_x;
   }else{
      ROBOT_speed.vel_x = joy_vel_x;
   }
   if(joy_vel_yaw == 0 && syn_vel_yaw != 0){
      ROBOT_speed.vel_yaw = syn_vel_yaw;
      turning_yaw = abs(syn_vel_yaw)*1.1;
      turning_radius = abs(syn_circle_R)-100;
   } else {
       ROBOT_speed.vel_yaw = joy_vel_yaw;
       turning_yaw = STATIC_YAW;
       turning_radius = STATIC_R;
   }
   vel_x = ROBOT_speed.vel_x;
   vel_yaw = ROBOT_speed.vel_yaw;
   vel_adjust_yaw = ROBOT_speed.adjust_yaw/2;
   ROS_INFO("the car liner speed is:%f, the car turnning speed is %f", vel_x, vel_yaw);

   if(vel_yaw ==0){
      Wheel_Speed[1].target_angle = 0;
      Wheel_Speed[0].target_angle = 0;
      // target_angle1 = 0;
      // target_angle2 = 0;
      // Wheel_Speed[0].liner_speed = vel_x;
      // Wheel_Speed[1].liner_speed = vel_x*-1;
      // Wheel_Speed[2].liner_speed = vel_x;
      // Wheel_Speed[3].liner_speed = vel_x*-1;
   }else if(vel_yaw > 0){
      Turning_Wheel_Cal(LEFT, turning_radius , turning_yaw);
   }else if(vel_yaw < 0) {
      Turning_Wheel_Cal(RIGHT, turning_radius , turning_yaw);
   }
   if(Raise_Car){
      Wheel_Speed[0].target_angle = -70.0f;
      Wheel_Speed[1].target_angle = 70.0f;
   }
   /*测试时，用临时变量，实际使用采用结构体中的target-angle*/
   yaw_angle1 = -PID_Position_Calculate(&YAW_PID, Wheel_Speed[0].target_angle, wheel_angle1);
   yaw_angle2 = -PID_Position_Calculate(&YAW_PID, Wheel_Speed[1].target_angle, wheel_angle2);
   // cout<<"yaw_angle1:"<<yaw_angle1<<endl;
   // cout<<"yaw_angle2:"<<yaw_angle2<<endl;
   // yaw_speed1 = Wheel_encoder_Calculate(yaw_angle1);
   Wheel_Speed[0].yaw_error_speed = yaw_angle1;
   // Wheel_Speed[1].yaw_error_speed = yaw_speed1;
   // yaw_speed2 = Wheel_encoder_Calculate(yaw_angle2);
   Wheel_Speed[1].yaw_error_speed = yaw_angle2;
   // Wheel_Speed[3].yaw_error_speed = yaw_speed2;
   // cout<<"yaw_angle1:"<<Wheel_Speed[2].yaw_error_speed<<endl;
   // cout<<"yaw_angle2:"<<Wheel_Speed[1].yaw_error_speed<<endl;

   // cout<<"the agv linear speed is:"<<vel_x<<"  ,the yaw speed is :"<<vel_yaw<<"  ,the adjust speed is :"<<vel_adjust_yaw<<endl;
   if( vel_x < 0 && vel_yaw != 0){
      Wheel_Speed[0].liner_speed  *= -1;
      Wheel_Speed[1].liner_speed  *= -1;
      // Wheel_Speed[2].liner_speed  *= -1;
      // Wheel_Speed[3].liner_speed  *= -1;
   }else if( vel_x >0 && vel_yaw != 0 ){
      Wheel_Speed[0].liner_speed  *= 1;
      Wheel_Speed[1].liner_speed  *= 1;
      // Wheel_Speed[2].liner_speed  *= 1;
      // Wheel_Speed[3].liner_speed  *= 1;
   }else{
      Wheel_Speed[0].liner_speed = vel_x - vel_adjust_yaw;
      Wheel_Speed[1].liner_speed = vel_x + vel_adjust_yaw;
      // Wheel_Speed[2].liner_speed = vel_x;
      // Wheel_Speed[3].liner_speed = vel_x*-1;
   }
   
   motor1_speed = (-Wheel_Speed[0].liner_speed)/(2*PI*0.05)*15.1*10000;
   motor2_speed = (Wheel_Speed[1].liner_speed)/(2*PI*0.05)*15.1*10000;
   motor3_speed = (Wheel_Speed[0].yaw_error_speed )*2.333*160*10000;
   motor4_speed = (Wheel_Speed[1].yaw_error_speed )*2.333*160*10000;
   // cout<<"the motor 1 speed is:"<<motor1_speed<<"  ,the motor 2 speed is :"<<motor2_speed<<endl;
   // cout<<"the motor 3 speed is:"<<motor3_speed<<"  ,the motor 4 speed is :"<<motor4_speed<<endl;
   // cout<<"the motor 3 speed is:"<<Wheel_Speed[2].liner_speed<<"  ,the motor 4 speed is :"<<Wheel_Speed[3].liner_speed<<endl;

   if(abs(motor1_speed) < 100){
       shell_data1[0] = 0x07;
   }
   if(abs(motor2_speed) < 100){
       shell_data2[0] = 0x07;
   }
   shell_data1[2] = motor1_speed&0xff;
   shell_data1[3] = (motor1_speed>>8)&0xff;
   shell_data1[4] = (motor1_speed>>16)&0xff;
   shell_data1[5] = (motor1_speed>>24)&0xff;
   shell_data2[2] = motor2_speed&0xff;
   shell_data2[3] = (motor2_speed>>8)&0xff;
   shell_data2[4] = (motor2_speed>>16)&0xff;
   shell_data2[5] = (motor2_speed>>24)&0xff;
   shell_data3[2] = motor3_speed&0xff;
   shell_data3[3] = (motor3_speed>>8)&0xff;
   shell_data3[4] = (motor3_speed>>16)&0xff;
   shell_data3[5] = (motor3_speed>>24)&0xff;
   shell_data4[2] = motor4_speed&0xff;
   shell_data4[3] = (motor4_speed>>8)&0xff;
   shell_data4[4] = (motor4_speed>>16)&0xff;
   shell_data4[5] = (motor4_speed>>24)&0xff;

   PDO_Send_func(slave_id1, 3, shell_data1, 8);
   PDO_Send_func(slave_id2, 3, shell_data2, 8);
   PDO_Send_func(slave_id3, 3, shell_data3, 8);
   PDO_Send_func(slave_id4, 3, shell_data4, 8);
}


void multiThreadListener::AgvVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // ROS_INFO_STREAM("ACCEPTING the jor control message!!!");
    static uint8_t Assistant_ARM = CLOSED;
    int motor1_speed, motor2_speed, motor3_speed, motor4_speed, motor5_speed,
        motor6_speed = 0;
    int slave_id1 = 1;
    int slave_id2 = 2;
    int slave_id3 = 3;
    int slave_id4 = 4;
    int slave_id5 = 5;
    int slave_id6 = 6;
    vector<unsigned char> shell_data1 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};
    vector<unsigned char> shell_data2 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};
    vector<unsigned char> shell_data3 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};
    vector<unsigned char> shell_data4 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};
    vector<unsigned char> shell_data5 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};
    vector<unsigned char> shell_data6 = {0x0f, 0, 0, 0, 0, 0, 0x03, 0};

    float vel_x, vel_yaw, vel_pole, vel_e_pole, yaw_angle1, yaw_angle2,
        yaw_speed1, yaw_speed2 = 0;
    float target_angle1, target_angle2 = 0;
    joy_vel_x= msg->linear.x / 20 * 5/2;
    joy_vel_yaw = msg->angular.z / 3;
    ROBOT_speed.adjust_yaw = msg->angular.x / 6;

    //  cout<<"vel_x:"<<ROBOT_speed.vel_x<<"  ,the vel_yaw speed is
    //  :"<<ROBOT_speed.vel_yaw<<endl;
    vel_pole = msg->linear.z;
    vel_e_pole = msg->linear.y;
    Raise_Car = msg->angular.y;

    // if(ROBOT_speed.vel_yaw ==0){
    //    Wheel_Speed[0].target_angle = 0;
    //    Wheel_Speed[1].target_angle = 0;
    //    Wheel_Speed[2].target_angle = 0;
    //    Wheel_Speed[3].target_angle = 0;
    //    target_angle1 = 0;
    //    target_angle2 = 0;
    // }else if(ROBOT_speed.vel_yaw ==1){
    //    Turning_Wheel_Cal(LEFT, 1500 , 10/180.0f*PI);
    // }else if(ROBOT_speed.vel_yaw == -1) {
    //    Turning_Wheel_Cal(RIGHT, 1500 , 10/180.0f*PI);
    // }else{
    //    Wheel_Speed[0].liner_speed = vel_x;
    //    Wheel_Speed[1].liner_speed = vel_x*-1;
    //    Wheel_Speed[2].liner_speed = vel_x;
    //    Wheel_Speed[3].liner_speed = vel_x*-1;
    // }
    // /*测试时，用临时变量，实际使用采用结构体中的target-angle*/
    // yaw_angle1 = PID_Position_Calculate(&YAW_PID,
    // Wheel_Speed[0].target_angle, wheel_angle1); yaw_angle2 =
    // PID_Position_Calculate(&YAW_PID, Wheel_Speed[3].target_angle,
    // wheel_angle2); yaw_speed1 = Wheel_encoder_Calculate(yaw_angle1);
    // Wheel_Speed[0].yaw_error_speed = yaw_speed1;
    // Wheel_Speed[1].yaw_error_speed = yaw_speed1;
    // yaw_speed2 = Wheel_encoder_Calculate(yaw_angle2);
    // Wheel_Speed[2].yaw_error_speed = yaw_speed2;
    // Wheel_Speed[3].yaw_error_speed = yaw_speed2;
    // cout<<"yaw_angle1:"<<Wheel_Speed[2].yaw_error_speed<<endl;
    // cout<<"yaw_angle2:"<<Wheel_Speed[1].yaw_error_speed<<endl;

    // // cout<<"the agv linear speed is:"<<vel_x<<"  ,the yaw speed is
    // :"<<vel_yaw<<endl;
    // // Wheel_Speed[0].liner_speed = vel_x*0;
    // // Wheel_Speed[1].liner_speed = vel_x*-1*0;
    // // Wheel_Speed[2].liner_speed = vel_x*0;
    // // Wheel_Speed[3].liner_speed = vel_x*-1*0;

    // motor1_speed = (Wheel_Speed[0].yaw_error_speed +
    // Wheel_Speed[0].liner_speed)/(2*PI*0.05)*5*10000; motor2_speed =
    // (Wheel_Speed[1].yaw_error_speed +
    // Wheel_Speed[1].liner_speed)/(2*PI*0.05)*5*10000; motor3_speed =
    // (Wheel_Speed[2].yaw_error_speed +
    // Wheel_Speed[2].liner_speed)/(2*PI*0.05)*5*10000; motor4_speed =
    // (Wheel_Speed[3].yaw_error_speed +
    // Wheel_Speed[3].liner_speed)/(2*PI*0.05)*5*10000; cout<<"the motor 1 speed
    // is:"<<motor1_speed<<"  ,the motor 2 speed is :"<<motor2_speed<<endl;
    // cout<<"the motor 3 speed is:"<<motor3_speed<<"  ,the motor 4 speed is
    // :"<<motor4_speed<<endl; shell_data1[2] = motor1_speed&0xff;
    // shell_data1[3] = (motor1_speed>>8)&0xff;
    // shell_data1[4] = (motor1_speed>>16)&0xff;
    // shell_data1[5] = (motor1_speed>>24)&0xff;
    // shell_data2[2] = motor2_speed&0xff;
    // shell_data2[3] = (motor2_speed>>8)&0xff;
    // shell_data2[4] = (motor2_speed>>16)&0xff;
    // shell_data2[5] = (motor2_speed>>24)&0xff;
    // shell_data3[2] = motor3_speed&0xff;
    // shell_data3[3] = (motor3_speed>>8)&0xff;
    // shell_data3[4] = (motor3_speed>>16)&0xff;
    // shell_data3[5] = (motor3_speed>>24)&0xff;
    // shell_data4[2] = motor4_speed&0xff;
    // shell_data4[3] = (motor4_speed>>8)&0xff;
    // shell_data4[4] = (motor4_speed>>16)&0xff;
    // shell_data4[5] = (motor4_speed>>24)&0xff;

    // PDO_Send_func(slave_id1, 3, shell_data1, 8);
    // PDO_Send_func(slave_id2, 3, shell_data2, 8);
    // PDO_Send_func(slave_id3, 3, shell_data3, 8);
    // PDO_Send_func(slave_id4, 3, shell_data4, 8);

    if (Left_Is_Arrival == MOVING) {
        shell_data5[0] = 0x0f;
    } else {
        shell_data5[0] = 0x07;
    }
    if (Right_Is_Arrival == MOVING) {
        shell_data6[0] = 0x0f;
    } else {
        shell_data6[0] = 0x07;
    }
    // cout<<"the motor 5 Actual_Pos is:"<<testMotor_info[4].Actual_Pos<<"  ,the
    // motor 6 Actual_Pos is :"<<testMotor_info[5].Actual_Pos<<endl;
    // cout<<"Left_Arm_status:"<<Left_Is_Arrival<<";Right_Arm_status:"<<Right_Is_Arrival<<endl;
    if (Assistant_ARM == OPENED && vel_pole == 1) {
        // cout<<"Left_Arm_status:"<<Left_Is_Arrival<<";Right_Arm_status:"<<Right_Is_Arrival<<endl;
        /*if the active arm is arrived, the motor should be disable while the
         * braking is closed*/
        if (Left_Is_Arrival == OPEN_ARRIVALED) {
            shell_data5[0] = 0x07;
        } else {
            shell_data5[0] = 0x0f;
        }
        if (Right_Is_Arrival == OPEN_ARRIVALED) {
            shell_data6[0] = 0x07;
        } else {
            shell_data6[0] = 0x0f;
        }
        motor5_speed = vel_pole * 400000;
        motor6_speed = vel_pole * 400000;
        if (testMotor_info[4].Actual_Pos >= 5000000) {
            motor5_speed *= 0.1;
        }
        if (testMotor_info[5].Actual_Pos >= 5000000) {
            motor6_speed *= 0.1;
        }
        // cout<<"the motor 5 speed is:"<<motor5_speed<<"  ,the motor 6 speed is
        // :"<<motor6_speed<<endl;
        cout << "the motor 5 Actual_Pos is:" << testMotor_info[4].Actual_Pos
             << "  ,the motor 6 Actual_Pos is :" << testMotor_info[5].Actual_Pos
             << endl;
        shell_data5[2] = motor5_speed & 0xff;
        shell_data5[3] = (motor5_speed >> 8) & 0xff;
        shell_data5[4] = (motor5_speed >> 16) & 0xff;
        shell_data5[5] = (motor5_speed >> 24) & 0xff;
        shell_data6[2] = motor6_speed & 0xff;
        shell_data6[3] = (motor6_speed >> 8) & 0xff;
        shell_data6[4] = (motor6_speed >> 16) & 0xff;
        shell_data6[5] = (motor6_speed >> 24) & 0xff;
    } else if (vel_pole == -1) {
        // cout<<"Left_Arm_status:"<<Left_Is_Arrival<<";Right_Arm_status:"<<Right_Is_Arrival<<endl;
        /*if the active arm is arrived, the motor should be disable while the
         * braking is closed*/
        if (Left_Is_Arrival == CLOSE_ARRIVALED) {
            shell_data5[0] = 0x07;
        } else {
            shell_data5[0] = 0x0f;
        }
        if (Right_Is_Arrival == CLOSE_ARRIVALED) {
            shell_data6[0] = 0x07;
        } else {
            shell_data6[0] = 0x0f;
        }
        motor5_speed = vel_pole * 400000;
        motor6_speed = vel_pole * 400000;
        if (testMotor_info[4].Actual_Pos <= 350000) {
            motor5_speed *= 0.2;
        }
        if (testMotor_info[5].Actual_Pos <= 350000) {
            motor6_speed *= 0.2;
        }
        // cout<<"the motor 5 speed is:"<<motor5_speed<<"  ,the motor 6 speed is
        // :"<<motor5_speed<<endl;
        cout << "the motor 5 Actual_Pos is:" << testMotor_info[4].Actual_Pos
             << "  ,the motor 6 Actual_Pos is :" << testMotor_info[5].Actual_Pos
             << endl;
        shell_data5[2] = motor5_speed & 0xff;
        shell_data5[3] = (motor5_speed >> 8) & 0xff;
        shell_data5[4] = (motor5_speed >> 16) & 0xff;
        shell_data5[5] = (motor5_speed >> 24) & 0xff;
        shell_data6[2] = motor6_speed & 0xff;
        shell_data6[3] = (motor6_speed >> 8) & 0xff;
        shell_data6[4] = (motor6_speed >> 16) & 0xff;
        shell_data6[5] = (motor6_speed >> 24) & 0xff;
    }
    PDO_Send_func(slave_id5, 3, shell_data5, 8);
    PDO_Send_func(slave_id6, 3, shell_data6, 8);

    // cout<<"the eletricity pole speed is:"<<vel_e_pole<<endl;
    if (vel_e_pole == 1) {
        Assistant_ARM = OPENED;
        ser_shell = {0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A};
    } else if (vel_e_pole == -1) {
        Assistant_ARM = CLOSED;
        ser_shell = {0x01, 0x05, 0x00, 0x01, 0xFF, 0x00, 0xDD, 0xFA};
    } else {
        ser_shell = {0x01, 0x0F, 0x00, 0x00, 0x00, 0x08,
                     0x02, 0x00, 0x00, 0xE4, 0x80};
    }
}

void multiThreadListener::SynVelotryCallback(const geometry_msgs::Twist::ConstPtr& msg) {
   ROS_INFO("RECEVING synergy info!!!");
   syn_vel_x = msg->linear.x;
   syn_circle_R = msg->angular.x*1000;
   syn_vel_yaw = msg->angular.z;
}

void multiThreadListener::AgvPositionCallback(const nav_msgs::Odometry::ConstPtr& Odo_msg)
{
   float AGV_pos_X, AGV_pos_Y, AGV_pos_Z;
   AGV_pos_X = Odo_msg->pose.pose.position.x;
   AGV_pos_Y = Odo_msg->pose.pose.position.y;
   AGV_pos_Z = Odo_msg->pose.pose.position.z;
   cout<<"the AGV now postion is "<<AGV_pos_X<<"m"<<AGV_pos_Y<<"m"<<AGV_pos_Z<<"m"<<endl;
}

void multiThreadListener::chatterCallback1(const pubmotor::motor::ConstPtr& msg)
{
  ROS_INFO("id: [%s]", msg->id.c_str());
}


void multiThreadListener::chatterCallback3(const can_msgs::Frame::ConstPtr &msg)
{
   static int times = 0;
	// std::cout << times++ << "ID: " << msg->id << " data: " ;    
   //  for (int i = 0; i < msg->dlc; i++)
   //  {
   //      printf("%X ", msg->data[i]);
   //  }
   CAN_msg_classify(msg->data.data(), msg->id,  msg->dlc);
}


void sigintHandler(int sig){
   multiThreadListener ur_msg;
   ur_msg.NMT_ShellFunc(0x00,0x81);
   ros::shutdown();
}

uint8_t serial_send(void){
   uint8_t Len = ser_shell.size();
   for(uint8_t i =0; i < Len; i++){
      s_buffer[i] = ser_shell[i];
      // printf("%02x",s_buffer[i]);
   }
   // printf("\r\n");
   return Len;
}

int main(int argc, char **argv)
{
   uint8_t rec =2;
   uint8_t len = 0;
  ros::init(argc, argv, "multi_rear_sub", ros::init_options::NoSigintHandler);

  multiThreadListener ur_msg;
  signal(SIGTERM, sigintHandler);
  signal(SIGINT, sigintHandler);

   serial::Serial ser;  //init the port IOpart
   try {
      ser.setPort("/dev/ttyS0");
      ser.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
   }
   catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port1 ");
      //return -1; //Even if the port can't open, the code still execute
   }
   if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port1 initialized for writing");
   } else {
      //return -1;
   }
   
   
   try {
      ser2.setPort("/dev/ttyS1");
      ser2.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser2.setTimeout(to);
      ser2.open();
   }
   catch (serial::IOException& e) {
      ROS_ERROR_STREAM("Unable to open port2 ");
      //return -1;
   }
   if(ser2.isOpen()){
      ROS_INFO_STREAM("Serial Port2 initialized for writing");
   } else {
      //return -1;
   }

   PID_Parameter_Speed_Init(&YAW_PID,0.005,0.0, 0.000007, 0.7, 0, 0, 1 );//这里修改了out limit
  vector<unsigned char> data = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00};
   ur_msg.NMT_ShellFunc(0x00,0x81);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
   ur_msg.NMT_ShellFunc(0x00,0x01);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));


   ur_msg.PDO_Send_func(1,3, data,8);
   ur_msg.PDO_Send_func(2,3, data,8);
   ur_msg.PDO_Send_func(3,3, data,8);
   ur_msg.PDO_Send_func(4,3, data,8);
   ur_msg.PDO_Send_func(5,3, data,8);
   ur_msg.PDO_Send_func(6,3, data,8);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));

   data[0] = 0x07;
   ur_msg.PDO_Send_func(1,3, data,8);
   ur_msg.PDO_Send_func(2,3, data,8);
   ur_msg.PDO_Send_func(3,3, data,8);
   ur_msg.PDO_Send_func(4,3, data,8);
   ur_msg.PDO_Send_func(5,3, data,8);
   ur_msg.PDO_Send_func(6,3, data,8);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
   data[0] = 0x0f;
   ur_msg.PDO_Send_func(1,3, data,8);
   ur_msg.PDO_Send_func(2,3, data,8);
   ur_msg.PDO_Send_func(3,3, data,8);
   ur_msg.PDO_Send_func(4,3, data,8);
   ur_msg.PDO_Send_func(5,3, data,8);
   ur_msg.PDO_Send_func(6,3, data,8);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ros::AsyncSpinner spinner(3); // Use 2 threads
  spinner.start();
  ros::Rate rate(30);
  ROS_INFO("Entering the loop!");
  while (ros::ok())
  {
      ur_msg.Adjust_Steering_WHEEL();
      rec = Request_encoder_data(rec);
      len = serial_send();
      ser.write(s_buffer,len);
      // ur_msg.PDO_Send_func(1,1, data,8);
      // ros::waitForShutdown();
      // ur_msg.Motor_Info(1);
      // ur_msg.Motor_Info(2);
      rate.sleep();
      ros::spinOnce();
  }
  return 0;
}
