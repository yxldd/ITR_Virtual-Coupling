/*
该版本的Arduino程序，用于实现：
1、舵机控制
2、电机控制
3、编码器读取
4、PID速度闭环
5、TCP通信传统
*/
#define encoderL 4
#define encoderR 5
#define motorL 12 //D6
#define motorR 14 //D5 
#define servopin 15 //D8
#define PI 3.1415926

#include <WiFi.h>
#include <Ticker.h>
#include <Servo.h>


const char* ssid     = "itrlab";
const char* password = "itrlab125";

typedef struct PID_Class{
    int SetPoint=0;
    double SumError=0,PrevError=0,LastError=0;
    double Proportion=0;
    double Integral=0;
    double Derivative=0;
};

PID_Class PID_ML,PID_MR;

void PID_set_(PID_Class &x, int Setpoint);
void PID_init_(PID_Class &x,double kp,double ki,double kd);
double PID_calc_(PID_Class &x, double NextPoint);

Ticker timer1;  // 中断函数
int interrupt_time = 100; // 中断时间
int timer_flag=0;     //计时器标志
volatile long Rcounter=0; // 右轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile 编码器转一圈加一
volatile long Lcounter=0; // 左轮脉冲计数  该变量用于存储编码器的值，所以用类型修饰符volatile 编码器转一圈加一
void right_counter_encoder(); // 右轮 计数脉冲数
void left_counter_encoder();  // 左轮 计数脉冲数  


int servopos=0;
int speedval=0;
int R,L;
double motconL,motconR;
Servo myservo;

void servocontrol();
void motorcontrol();
void TCP_Communication();

void setup()
{
/*****************  舵机、马达初始化  *****************/
    myservo.attach(servopin);
    pinMode(motorL,OUTPUT);
    pinMode(motorR,OUTPUT);
/*****************  WIFI初始化  *****************/

/*****************  编码器初始化  *****************/
    pinMode(encoderL, INPUT);       
    pinMode(encoderR, INPUT);    
    attachInterrupt(encodeR, right_counter_encoder, RISING);//设置编码器R相位上升沿中断
    attachInterrupt(encodeL, left_counter_encoder, RISING);//设置编码器L相位上升沿中断
/*****************  定时中断  ********************/   
    timer1.attach_ms(interrupt_time, timerIsr);// 打开定时器中断
    interrupts();//打开外部中断  
}

void loop()
{
    if(timer_flag==1)
    {
        timer_flag=0;
        motconL=PIDCal(PID_ML,R);
        motconR=PIDCal(PID_MR,L);
    }
    servocontrol();
    motorcontrol();
}

void servocontrol()
{
    myservo.write(servopos);
}

/*********  编码器相关函数 *******/
void timerIsr(){
    timer_flag=1;
    readEncoder();
}
void readEncoder(){
    Serial.print("编码器输出：  ");  
    Serial.println(Lcounter);
    //数值清零，重新计数
    R=Rcounter;
    L=Lcounter;
    Rcounter = 0;    
    Lcounter = 0; 
}
void right_counter_encoder(){  //右轮计数
    Rcounter++;
}
void left_counter_encoder(){  // //左轮计数
    Lcounter++;
}

/**********  PID相关函数  *********/
void PID_set_(PID_Class &x, double Setpoint){
    x.SetPoint=Setpoint;
}
void PID_init_(PID_Class &x,double kp,double ki,double kd)
{
    x.Derivative=kd;
    x.Integral=ki;
    x.Proportion=kp;
} 
double PID_calc_(PID_Class &x, double NextPoint){
    double DError,Error;
    Error=x.SetPoint-NextPoint;
    DError=x.LastError-x.PrevError;
    x.SumError+=Error;
    x.PrevError=x.LastError;
    x.LastError=x.Error;
    return (x.Proportion*Error+x.Integral*x.SumError+x.Derivative*DError);
}
void motorcontrol()
{
    analogWrite(motor1,motorL);
    analogWrite(motor1,motorR);
}