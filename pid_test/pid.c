#include <main.h>
#include "pid.h"
void PID_init(){               //PID初始化函数，将所有值归零
   
    pid.Set=0.0;
    pid.Actual=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.action=0.0;
    pid.integral=0.0;
    pid.Kp=0.0;
    pid.Ki=0.0;
    pid.Kd=0.0;
    
}
float PID_realize(float target, float kp,float ki,float kd){
    pid.Set=target;   //target即为控制系统的目标值
    pid.Kp=kp;       //在此设定PID三个需要调节的参数
    pid.Ki=ki;
    pid.Kd=kd;
    pid.err=pid.Set-pid.Actual;     //误差等于目标值减去实际值
    pid.integral+=pid.err;       //对误差积分
 pid.action=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);  //PID公式
    pid.err_last=pid.err;         //更新误差
    pid.Actual=pid.action*1.0;
    return pid.Actual;          //返回结果PID控制处理后的值
}