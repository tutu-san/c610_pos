#pragma once
#ifndef ROBOMAS_ENCODER
#define ROBOMAS_ENCODER

#include <cstdint>

class robomas_encoder{
private:
    uint8_t robomas_input_rpm_high = 0, robomas_input_rpm_low = 0; //save robomas_encoder_data  
public:
    void input_encoder_data(uint8_t[8]);
    int show_rpm();
};

#endif