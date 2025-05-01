//
// Created by Lenovo on 2024/3/15.
//

#ifndef KOSANN_UAVGIMBAL_SLIDING_HPP
#define KOSANN_UAVGIMBAL_SLIDING_HPP

#include <cmath>
#include "DebugC.h"

#define SAMPLE_PERIOD 0.002
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

typedef struct {

    float tar_now;//当前目标值
    float tar_last;//上一次目标值
    float tar_differential;//目标值一阶导
    float tar_differential_last;//上一次目标值一阶导
    float tar_differential_second;//目标值二阶导

    float pos_get;//当前位置
    float vol_get;//当前速度

    float p_error;//位置误差
    float v_error;//速度误差（位置误差一阶导）

    float p_error_integral;//位置误差积分
    float v_error_integral;//速度误差积分
//    float v_error_integral_max; //积分限幅

    float pos_error_eps;   //误差精度
    float vol_error_eps;   //误差精度
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;   //EIsmc参数
    float c2;   //EIsmc参数

    float p;    //tfsmc参数，正奇数 p>q
    float q;    //tfsmc参数，正奇数
    float beta; //tfsmc参数，正数
    float epsilon; //ε噪声上限
}SlidingParam;

typedef struct {
    float u; //控制器输出
    float s; //滑模面计算储存

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max; //输出限幅
    Rmode flag; //符号和饱和切换，未用到
    float limit; //饱和函数上下限
}Sliding;


class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //滑膜参数设定 EXPONENT,POWER,VELSMC 参数
    void SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//滑膜参数设定 TFSMC 参数
    void SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC 参数设定

    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜位置误差更新
    void ErrorUpdate(float target,float vol_now); //滑模速度误差更新
    void Clear();
    void Integval_Clear();

    float SmcCalculate(); //滑模控制器计算函数
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //参量提取

private:


    Sliding smc;
    void OutContinuation(); //输出连续化
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};
#endif //KOSANN_UAVGIMBAL_SLIDING_HPP
