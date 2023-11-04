#ifndef __PID_H
#define __PID_H
struct _pid{            //ʵ��ʹ��ʱ���ཫ���ṹ�嶨����.h�ļ���
    float Set;            //�����趨ֵ
    float Actual;        //����ʵ��ֵ���ɴ��������������ȵȣ��õ�
    float err;                //���嵱ǰƫ��ֵ
    float err_last;            //������һ��ƫ��ֵ
    float Kp,Ki,Kd;            //������������֡�΢��ϵ��
    float action;          //�������ִ�����ı���������ֱ�Ӷ�ִ�������в���
    float integral;            //�������ֵ
}pid;
float PID_realize(float target, float kp,float ki,float kd);
void PID_init();


#endif