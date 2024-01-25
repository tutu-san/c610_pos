#include "robomas_encoder.hpp"

void robomas_encoder::input_encoder_data(uint8_t robomas_encoder_data[8]){
    //データを入れるってこと
    robomas_input_rpm_high = robomas_encoder_data[2];
    robomas_input_rpm_low = robomas_encoder_data[3];
}

int robomas_encoder::show_rpm(){
    //上位,下位ビットの統合
    uint16_t unsigned_robomas_rpm_data = robomas_input_rpm_high << 8 | robomas_input_rpm_low;

    int16_t signed_robomas_rpm_data = 0;

    signed_robomas_rpm_data = unsigned_robomas_rpm_data;
    signed_robomas_rpm_data /= 36; //ギア比で割っている

    //debug
    debug_robomas_rpm = signed_robomas_rpm_data;
    //おしまい
    return signed_robomas_rpm_data;
}
