#include <main.h>
#include "pid.h"
void PID_init(){               //PID��ʼ��������������ֵ����
   
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
    pid.Set=target;   //target��Ϊ����ϵͳ��Ŀ��ֵ
    pid.Kp=kp;       //�ڴ��趨PID������Ҫ���ڵĲ���
    pid.Ki=ki;
    pid.Kd=kd;
    pid.err=pid.Set-pid.Actual;     //������Ŀ��ֵ��ȥʵ��ֵ
    pid.integral+=pid.err;       //��������
 pid.action=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);  //PID��ʽ
    pid.err_last=pid.err;         //�������
    pid.Actual=pid.action*1.0;
    return pid.Actual;          //���ؽ��PID���ƴ�����ֵ
}