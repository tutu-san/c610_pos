#include "user_code.hpp"
/*main_functions*/
void setup(){
    can_setup();
}

void loop(){
    // float motor_pwm[3]; //お守り(メモ):いざとなったら、中身を確認すること
	for(int i=0; i<3; i++){
		pid[i].update_target_speed(-200.0f);
	}
    for(int i = 0; i<3; i++){
        //ここでロボマスからの上位・下位ビットをくっつける -> piの計算 -> ロボマスpwmデータを入れる
    	robomas.input_rotation_data(i, pid[i].motor_calc(encoders[i].show_rpm()));
//    	encoders[i].show_rpm();
//    	robomas.input_rotation_data(i, 200);
    }
    robomas.rotate(); //ロボマスpwmデータを送信
    HAL_Delay(1000);
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
        for(int i=0; i<3; i++){
            pid[i].update_target_speed((float)can1_rx_data[i]);
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
