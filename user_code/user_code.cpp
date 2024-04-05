#include "user_code.hpp"
/*main_functions*/
void setup(){
    can_setup();
    encoders[0].first_axis_pos();
}

void loop(){
    //debug set_speed (deg)
	 for(int i=0; i<3; i++){
	 	pid[i].update_target_speed(180.0f);
	 }
    for(int i = 0; i<3; i++){
        //ここでロボマスからの上位・下位ビットをくっつける -> piの計算 -> ロボマスpwmデータを入れる
//    	int robomas_rpm[3] = {150,0,0};
//    	robomas_rpm[i] = encoders[i].show_rpm();
    	robomas.input_rotation_data(i, pid[i].motor_calc((float)(encoders[i].show_pos())));
        //degug set_pwm_data
   	    //encoders[i].show_rpm();
//   	    robomas.input_rotation_data(i, 0);
//   	    encoders[i].show_pos();
//   	    if(encoders[0].show_pos() > 360.0f){
//   	    	test_pwm = 0;
//   	    }
//   	    encoders[i].reset_positon(20);
    }
    robomas.rotate(); //ロボマスpwmデータを送信
}

/*can_functions*/
void can_setup(){
//FDCAN1
	FDCAN_FilterTypeDef  sFilterConfig1;
	sFilterConfig1.IdType = FDCAN_STANDARD_ID;
	sFilterConfig1.FilterIndex = 0;
	sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig1.FilterID1 = 0x000;
	sFilterConfig1.FilterID2 = 0x000;

	HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1);
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

//FDCAN2
	FDCAN_FilterTypeDef  sFilterConfig2;
	sFilterConfig2.IdType = FDCAN_STANDARD_ID;
	sFilterConfig2.FilterIndex = 0;
	sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig2.FilterID1 = 0x000;
	sFilterConfig2.FilterID2 = 0x000;

	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2);
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void can1_receive_process(){
    //from pc
    if(can1_rx_id == 0x300){
    	int rpm_data_from_pc[3] = {0,0,0};
    	rpm_data_from_pc[0] = pcdata_to_rpm(can1_rx_data[0], can1_rx_data[1]);
    	rpm_data_from_pc[1] = pcdata_to_rpm(can1_rx_data[2], can1_rx_data[3]);
    	rpm_data_from_pc[2] = pcdata_to_rpm(can1_rx_data[4], can1_rx_data[5]);
        for(int i=0; i<3; i++){
        	pid[i].update_target_speed((float)rpm_data_from_pc[i]);
        }
    }

}

void can2_receive_process(){
    //from robomas_encoder
    switch(can2_rx_id){
    case 0x201:
        encoders[0].input_encoder_data(can2_rx_data);
        break;
    case 0x202:
        encoders[1].input_encoder_data(can2_rx_data);
        break;
    case 0x203:
        encoders[2].input_encoder_data(can2_rx_data);
        break;
    default:
        break;
    }
}

int pcdata_to_rpm(uint8_t pc_input_data_high, uint8_t pc_input_data_low){
    //上位,下位ビットの統合
    uint16_t unsigned_rpm_data = pc_input_data_high << 8 | pc_input_data_low;
    int16_t signed_rpm_data = 0;

    signed_rpm_data = unsigned_rpm_data;

    //おしまい
    return signed_rpm_data;
}
