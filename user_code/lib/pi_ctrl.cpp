#include "pi_ctrl.hpp"

//単位は(rpm)
float pid_control::pi_calc(float current_speed){
    float result_p = 0, result_i = 0;
    float error = 0;
    float pi_result = 0;

    //計算
    error = current_target_speed - current_speed;
    integral += error * DELTA_T;
    pid_control::clamp(integral, -10000.0f, 10000.0f); //値は適当(どのくらいがいいのかわからない)

    //PIの各成分を出す
    result_p = gain_p * error;
    result_i = gain_i * integral;

    pi_result = result_p + result_i;
    pid_control::clamp(pi_result, -14000.0f, 14000.0f);
    return pi_result;
}

//現在の目標速度設定(rpmで来る)
void pid_control::update_target_speed(float input_speed){
    float checked_new_speed = 0.0f;

	this->current_target_speed = checked_new_speed;
}

//エンコーダーの入力から、PWMの値を求める
float pid_control::motor_calc(float meter_per_sec){
    float result_motor_pwm = 0.0f;

    result_motor_pwm = this->pi_calc(meter_per_sec);

    return result_motor_pwm;
}

//インテグラルをリセットする(非常停止解除前に実行すること)
void pid_control::reset_integral(){
	this->integral = 0.0f;
}

//std::clampちゃんが起きないので
void pid_control::clamp(float&terget_variable , float min, float max){
    if(terget_variable < min){
        terget_variable = min;
        return;
    }else if(terget_variable > max){
        terget_variable = max;
        return;
    }else{
        return;
    }
}
