/**
 * @file pi_ctrl.hpp(v 1.1 / for robomas_encoder)
 * @brief PI制御の計算
 * @author tutu-san
 * @details PI制御に必要な比例・積分計算を行う
 * @date 2024-1-10
 */
#pragma once

#ifndef PI_CTRL_HPP
#define PI_CTRL_HPP

//include
#include <cstdint>

class pid_control{
private:
    //計算時使用の定数
    float gain_p;
    float gain_i;
    float gain_d; //未実装のD制御
    static constexpr float DELTA_T = 0.01f;

    float current_target_speed = 0.0f;
    float integral = 0.0f; //累積値保存, I制御に使う

    float result_motor_pwm = 0.0f;

    float pi_calc(float);
    void clamp(float&, float, float);
public:
    pid_control(float _gain_p, float _gain_i):gain_p(_gain_p), gain_i(_gain_i){}
    void update_target_speed(float);
    float motor_calc(float);
    void reset_integral();
};

#endif
