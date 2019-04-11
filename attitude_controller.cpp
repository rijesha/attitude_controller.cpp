#include "attitude_controller.h"

std::chrono::time_point last_run_time = std::chrono::high_resolution_clock::now();
std::chrono::time_point current_run_time = std::chrono::high_resolution_clock::now();
std::chrono::duration<float> elapsed = current_run_time - last_run_time;

int i = 0;

void AttitudeController::calc_average_velocity(){
    vel_avg[avg_ind % 3] = ned_vel_curr;
    if (reinitialize_state || i < AVGS) {
        if (reinitialize_state) i = 0;
        i++;
        ned_vel_curr_avg = ned_vel_curr;
    } else {
        ned_vel_curr_avg = (vel_avg[0] + vel_avg[1] + vel_avg[2])/3;
    }
    avg_ind++;
}

void AttitudeController::update_dt()  {
    current_run_time = std::chrono::high_resolution_clock::now();
    elapsed = current_run_time - last_run_time;
    dt = elapsed.count();
    last_run_time = current_run_time;

    reinitialize_state = (dt < RESET_STATE_DT) ? false : true;
}

void AttitudeController::update_velocity_state()  {
    if (!reinitialize_state){
        ned_vel_curr = (ned_pos_curr - ned_pos_last)/dt;
    }
    ned_pos_last = ned_pos_curr;
}

void AttitudeController::update_velocity_err_integration()  {
    if (reinitialize_state) {
        ned_vel_int = {0, 0, 0};
    } else {
        ned_vel_int += ned_vel_err*dt;
    }
}

float AttitudeController::bind_max_value(float val, float max_val){
    if (val > max_val)
        return max_val;
    else if (val < -max_val)
        return -max_val;
    return val;
}

float AttitudeController::bind_max_value(float val, float max_val, float min_val){
    if (val > max_val)
        return max_val;
    else if (val < min_val)
        return min_val;
    return val;
}

void AttitudeController::updateQuaternion(){
    float cr2 = cosf(roll_target*M_PI/180.0f*0.5f);
    float cp2 = cosf(pitch_target*M_PI/180.0f*0.5f);
    float cy2 = cosf(yaw_target*M_PI/180.0f*0.5f);
    float sr2 = sinf(roll_target*M_PI/180.0f*0.5f);
    float sp2 = sinf(pitch_target*M_PI/180.0f*0.5f);
    float sy2 = sinf(yaw_target*M_PI/180.0f*0.5f);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

void AttitudeController::run_loop(float curr_x, float curr_y, float curr_z, float desir_x, float desir_y, float desir_z, float desir_yaw){
    update_dt();

    ned_pos_curr = {curr_x, curr_y, curr_z};
    
    //updating current velocity
    update_velocity_state();
    calc_average_velocity();

    //Calculating position error
    ned_pos_desi = {desir_x, desir_y, desir_z};
    ned_pos_err = ned_pos_desi - ned_pos_curr;

    //Calculating desired velocity
    ned_vel_desi = ned_pos_err.multiplyGain(pos_pgain);

    //bind velocity to max
    ned_vel_desi.bindToMaxVal(max_vel);

    //calculating velocity error
    ned_vel_err = ned_vel_desi - ned_vel_curr_avg;

    //updating velocity error integration
    update_velocity_err_integration();

    //calculating desired acceleration
    ned_acc_desi = ned_vel_err.multiplyGain(vel_pgain) + ned_vel_int.multiplyGain(vel_igain);

    pitch_target = atanf(-ned_acc_desi.x/(9.806))*(180.0f/M_PI);
    cos_pitch_target = cosf(pitch_target*M_PI/180.0f);
    roll_target = atanf(ned_acc_desi.y*cos_pitch_target/(9.806))*(180.0f/M_PI);

    //climb_rate_cms = (packet.thrust - 0.5f) * 2.0f * copter.wp_nav->get_default_speed_up();
    thrust = (ned_vel_desi.z*100)/(2.0*10) + 0.5;
    thrust = bind_max_value(thrust,1,0);

    pitch_target = bind_max_value(pitch_target, 10);
    roll_target = bind_max_value(roll_target, 10);
    yaw_target = desir_yaw;

    updateQuaternion();

    if (!reinitialize_state){
        //send message

    }
    reinitialize_state = false;
}

string AttitudeController::get_state_string(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << ned_pos_curr.x << ',' << ned_pos_curr.y << ',' << ned_pos_curr.z;
    output << ned_pos_last.x << ',' << ned_pos_last.y << ',' << ned_pos_last.z;
    output << ned_pos_err.x  << ',' << ned_pos_err.y  << ',' << ned_pos_err.z;
    output << ned_pos_desi.x << ',' << ned_pos_desi.y << ',' << ned_pos_desi.z;
    output << ned_vel_curr.x << ',' << ned_vel_curr.y << ',' << ned_vel_curr.z;
    output << ned_vel_last.x << ',' << ned_vel_last.y << ',' << ned_vel_last.z;
    output << ned_vel_desi.x << ',' << ned_vel_desi.y << ',' << ned_vel_desi.z;
    output << ned_vel_err.x  << ',' << ned_vel_err.y  << ',' << ned_vel_err.z;
    output << ned_vel_int.x  << ',' << ned_vel_int.y  << ',' << ned_vel_int.z;
    output << ned_acc_desi.x << ',' << ned_acc_desi.y << ',' << ned_acc_desi.z;
    
    output << pitch_target << ',';
    output << roll_target << ',';   
    output << yaw_target << ',';
    output << thrust << ',' << endl;
    return output.str();
}


//To do
//rotate frame
//send message code
//test what happens if the uav doesn't recevie messages frequently enough.
//Have fun in life.
//Set wp speed up to 10cm/s or change thrust math
