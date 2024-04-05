#include "robomas_encoder.hpp"

void robomas_encoder::input_encoder_data(uint8_t robomas_encoder_data[8]){
    //データを入れるってこと
    robomas_input_rpm_high = robomas_encoder_data[2];
    robomas_input_rpm_low = robomas_encoder_data[3];
    robomas_input_pos_high = robomas_encoder_data[0];
    robomas_input_pos_low = robomas_encoder_data[1];
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

float robomas_encoder::show_pos(){
	//上位,下位ビットの統合
	uint16_t unsigned_robomas_pos_data = robomas_input_pos_high << 8 | robomas_input_pos_low;
//	int16_t signed_robomas_pos_data = 0;
//	float signed_robomas_pos_axis_data = 0.0f;

	signed_robomas_pos_data = unsigned_robomas_pos_data;
	signed_robomas_pos_axis_data = ((static_cast<float>(signed_robomas_pos_data) / 22.75f) / 36.0f) - 5.0f;

	const bool gear_rotation_check_0 = signed_robomas_pos_axis_data > 0 && save_before_axis_pos < 0;
	const bool gear_rotation_check_1 = signed_robomas_pos_axis_data < 0 && save_before_axis_pos > 0;
	const bool gear_rotation_check_2 = signed_robomas_pos_axis_data > 2.5f && save_before_axis_pos < -2.5f;
	const bool gear_rotation_check_3 = signed_robomas_pos_axis_data < -2.5f && save_before_axis_pos > 2.5f;
	if(gear_rotation_check_0 && gear_rotation_check_2){
		float high_gap = 5.0f - signed_robomas_pos_axis_data;
		float low_gap = save_before_axis_pos - (-5.0f);
		debug_robomas_pos = high_gap + low_gap;
	}else if(gear_rotation_check_1 && gear_rotation_check_3){
		float high_gap = 5.0f - save_before_axis_pos;
		float low_gap = signed_robomas_pos_axis_data - (-5.0f);
		debug_robomas_pos = high_gap + low_gap;
	}else{
		debug_robomas_pos = signed_robomas_pos_axis_data - save_before_axis_pos;
	}
	save_before_axis_pos = signed_robomas_pos_axis_data;
	static int counter = 0;
	if (counter != 0) ruisekiwa += debug_robomas_pos;
	counter++;
	return ruisekiwa;
}

void robomas_encoder::first_axis_pos(){
	//上位,下位ビットの統合
	uint16_t unsigned_robomas_pos_data = robomas_input_pos_high << 8 | robomas_input_pos_low;
//	int16_t signed_robomas_pos_data = 0;
	float signed_robomas_pos_axis_data = 0.0f;

	signed_robomas_pos_data = unsigned_robomas_pos_data;
	signed_robomas_pos_axis_data = (static_cast<float>(signed_robomas_pos_data) / 36.0f);
	save_before_axis_pos = signed_robomas_pos_axis_data;
	return;
}

void robomas_encoder::reset_positon(int new_default_position){
	default_position = new_default_position;
}
