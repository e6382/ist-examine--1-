#ifndef __PID_H
#define __PID_H
struct _pid{            //实际使用时更多将本结构体定义在.h文件内
    float Set;            //定义设定值
    float Actual;        //定义实际值，由传感器（编码器等等）得到
    float err;                //定义当前偏差值
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float action;          //定义控制执行器的变量，用于直接对执行器进行操作
    float integral;            //定义积分值
}pid;
float PID_realize(float target, float kp,float ki,float kd);
void PID_init();


#endif