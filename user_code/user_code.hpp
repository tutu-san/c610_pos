#pragma once
#ifndef USER_CODE
#define USER_CODE

//includes
#include <cstdint>
#include "stm32h7xx_hal.h"
#include "lib/robomas_encoder.hpp"
#include "lib/robomas.hpp"
#include "lib/pi_ctrl.hpp"

//(extern) variables, classes
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

extern uint32_t can1_rx_id;
extern uint8_t can1_rx_data[8];
extern uint32_t can2_rx_id;
extern uint8_t can2_rx_data[8];

extern robomas_encoder encoders[3];
extern pid_control pid[3];
extern robomas_rotation robomas;
//functions
void setup();
void loop();

void can_setup();
void can1_receive_process();
void can2_receive_process();
#endif
