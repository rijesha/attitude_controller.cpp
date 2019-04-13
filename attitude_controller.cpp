#include "attitude_controller.h"

std::chrono::time_point last_run_time = std::chrono::high_resolution_clock::now();
std::chrono::time_point current_run_time = std::chrono::high_resolution_clock::now();
std::chrono::duration<float> elapsed = current_run_time - last_run_time;

int i = 0;

void AttitudeController::calc_average_velocity(){
    vel_avg[avg_ind % 3] = vel_curr;
    if (reinitialize_state || i < AVGS) {
        if (reinitialize_state) i = 0;
        i++;
        vel_curr_avg = vel_curr;
    } else {
        vel_curr_avg = (vel_avg[0] + vel_avg[1] + vel_avg[2])/3;
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
        vel_curr = (pos_curr - pos_last)/dt;
    }
    pos_last = pos_curr;
}

void AttitudeController::update_velocity_err_integration()  {
    if (reinitialize_state) {
        vel_int = {0, 0, 0};
    } else {
        vel_int += vel_err*dt;
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

void AttitudeController::run_loop(Vector3f current_pos, Vector3f desired_pos){
    update_dt();

    pos_curr = current_pos;
    
    //updating current velocity
    update_velocity_state();
    calc_average_velocity();

    //Calculating position error
    pos_desi = desired_pos;
    pos_err = pos_desi - pos_curr;

    //Calculating desired velocity
    vel_desi = pos_err.multiplyGain(pos_pgain);

    //bind velocity to max
    vel_desi.bindToMaxVal(max_vel);

    //calculating velocity error
    vel_err = vel_desi - vel_curr_avg;

    //updating velocity error integration
    update_velocity_err_integration();

    //calculating desired acceleration
    acc_desi = vel_err.multiplyGain(vel_pgain) + vel_int.multiplyGain(vel_igain);
    reinitialize_state = false;
}

void AttitudeController::rotateAccelerations(float acc_x, float acc_y, float current_yaw){
    float acc_x_new = -acc_x * cos(current_yaw) - acc_y * sin(current_yaw);
    float acc_y_new = -acc_x * sin(current_yaw) + acc_y * cos(current_yaw);
}

void AttitudeController::acceleration_to_attitude(float forward_acc, float right_acc, float desir_yaw){
    pitch_target = atanf(-forward_acc/(9.806))*(180.0f/M_PI);
    cos_pitch_target = cosf(pitch_target*M_PI/180.0f);
    roll_target = atanf(right_acc*cos_pitch_target/(9.806))*(180.0f/M_PI);
    
    pitch_target = bind_max_value(pitch_target, MAX_ANGLE);
    roll_target = bind_max_value(roll_target, MAX_ANGLE);
    yaw_target = desir_yaw;

    updateQuaternion();
}

Vector3f AttitudeController::get_desired_velocity(){
    return vel_desi;
}

string AttitudeController::get_state_string(){
    stringstream output;
    output << std::fixed;
    output << std::setprecision(5);

    output << pos_curr.x << ',' << pos_curr.y << ',' << pos_curr.z;
    output << pos_last.x << ',' << pos_last.y << ',' << pos_last.z;
    output << pos_err.x  << ',' << pos_err.y  << ',' << pos_err.z;
    output << pos_desi.x << ',' << pos_desi.y << ',' << pos_desi.z;
    output << vel_curr.x << ',' << vel_curr.y << ',' << vel_curr.z;
    output << vel_last.x << ',' << vel_last.y << ',' << vel_last.z;
    output << vel_desi.x << ',' << vel_desi.y << ',' << vel_desi.z;
    output << vel_err.x  << ',' << vel_err.y  << ',' << vel_err.z;
    output << vel_int.x  << ',' << vel_int.y  << ',' << vel_int.z;
    output << acc_desi.x << ',' << acc_desi.y << ',' << acc_desi.z;
    
    output << pitch_target << ',';
    output << roll_target << ',';   
    output << yaw_target << ',';
    output << thrust << ',' << endl;
    return output.str();
}


//To do
//test what happens if the uav doesn't recevie messages frequently enough.
//Have fun in life.
//Set wp speed up to 10cm/s or change thrust math
//check avg velcotiy calc