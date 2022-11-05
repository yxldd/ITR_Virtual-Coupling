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
#include <ESP32Servo.h>


const char* ssid     = "itrlab";
const char* password = "itrlab125";
const IPAddress serverIP(192,168,3,185); //欲访问的地址
uint16_t serverPort = 11411;         //服务器端口号

WiFiClient client; //声明一个客户端对象，用于与服务器进行连接

typedef struct PID_Class{
    int SetPoint=0;
    double SumError=0,PrevError=0,LastError=0;
    double Proportion=0;
    double Integral=0;
    double Derivative=0;
};

PID_Class PID_ML,PID_MR;

void PID_set_(PID_Class *x, int Setpoint);
void PID_init_(PID_Class *x,double kp,double ki,double kd);
double PID_calc_(PID_Class *x, double NextPoint);

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
int motconL,motconR;
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
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); //关闭STA模式下wifi休眠，提高响应速度
    WiFi.begin(ssid, password);
    Serial.begin(115200);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
/*****************  编码器初始化  *****************/
    pinMode(encoderL, INPUT);       
    pinMode(encoderR, INPUT);  
    PID_init_(&PID_ML,1,0,0);
    PID_init_(&PID_MR,1,0,0);  
    attachInterrupt(encoderR, right_counter_encoder, RISING);//设置编码器R相位上升沿中断
    attachInterrupt(encoderL, left_counter_encoder, RISING);//设置编码器L相位上升沿中断
/*****************  定时中断  ********************/   
    timer1.attach_ms(interrupt_time, timerIsr);// 打开定时器中断
    interrupts();//打开外部中断  
}

void loop()
{
    TCP_Communication();
    PID_set_(&PID_ML,speedval);
    PID_set_(&PID_MR,speedval);
    if(timer_flag==1)
    {
        timer_flag=0;
        motconL=PID_calc_(&PID_ML,R);
        motconR=PID_calc_(&PID_MR,L);
    }
    servocontrol();
    motorcontrol();
    servopos=0;
    speedval=0;
}

void servocontrol()
{
    myservo.write(servopos);
}
void TCP_Communication()
{
    if (client.connect(serverIP, serverPort)) //尝试访问目标地址
    {
        while (client.connected() || client.available()) //如果已连接或有收到的未读取的数据
        {
            if (client.available()) //如果有数据可读取
            {
                String msg = client.readStringUntil('\n'); //读取数据到换行符
                /*  
                //串口调试使用
                Serial.print("读取到数据：");
                Serial.println(msg);
                client.write(msg.c_str()); //将收到的数据回发
                */
                int i=0;
                for( ;msg[i]!='a';i++)
                {
                    servopos=servopos*10+(msg[i]-'0');
                }
                i++;
                for( ;msg[i]!='\0';i++)
                {
                    speedval=speedval*10+(msg[i]-'0');
                }
            }
        }
    }
}


void timerIsr(){
    timer_flag=1;
    readEncoder();
}
void readEncoder(){
    //Serial.print("编码器输出：  ");  
    //Serial.println(Lcounter);
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
void PID_set_(PID_Class *x, int Setpoint){
    x->SetPoint=Setpoint;
}
void PID_init_(PID_Class *x,double kp,double ki,double kd)
{
    x->Derivative=kd;
    x->Integral=ki;
    x->Proportion=kp;
} 
double PID_calc_(PID_Class *x, double NextPoint){
    double DError,Error;
    Error=x->SetPoint-NextPoint;
    DError=x->LastError-x->PrevError;
    x->SumError+=Error;
    x->PrevError=x->LastError;
    x->LastError=Error;
    return (x->Proportion*Error+x->Integral*x->SumError+x->Derivative*DError);
}
void motorcontrol()
{
    analogWrite(motorL,motconL);
    analogWrite(motorR,motconR);
}