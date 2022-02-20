#include "position_controller.h"

#define MAX_ANGLE 2
#define MAX_HORZ_VEL 0.3
#define MAX_VERT_VEL 0.3

void PositionController::calc_average_velocity() {
  vel_avg[vel_avg_ind % 3] = vel_curr;
  if (reinitialize_state || vel_avg_i_ < VEL_AVGS) {
    if (reinitialize_state) vel_avg_i_ = 0;
    vel_avg_i_++;
    vel_curr_avg = vel_curr;
  } else {
    for (int i = 0; i < VEL_AVGS; i++) {
      vel_curr_avg = vel_avg[i];
    }
    vel_curr_avg = vel_curr_avg / VEL_AVGS;
  }
  vel_avg_ind++;
}

void PositionController::calc_average_position() {
  pos_avg[pos_avg_ind % 3] = pos_curr;
  if (reinitialize_state || pos_avg_i_ < POS_AVGS) {
    if (reinitialize_state) pos_avg_i_ = 0;
    pos_avg_i_++;
    pos_curr_avg = pos_curr;
  } else {
    for (int i = 0; i < POS_AVGS; i++) {
      pos_curr_avg = pos_avg[i];
    }
    pos_curr_avg = pos_curr_avg / POS_AVGS;
  }
  pos_avg_ind++;
}

void PositionController::update_dt() {
  current_run_time = std::chrono::high_resolution_clock::now();
  elapsed = current_run_time - last_run_time;
  dt = elapsed.count();
  last_run_time = current_run_time;

  reinitialize_state = (dt < RESET_STATE_DT) ? false : true;
}

void PositionController::update_velocity_state() {
  if (!reinitialize_state) {
    vel_curr = (pos_curr - pos_last) / dt;
  }
  pos_last = pos_curr;
}

void PositionController::update_velocity_err_integration() {
  if (reinitialize_state) {
    vel_int = {0, 0, 0};
  } else {
    vel_int += vel_err * dt;
  }
}

float PositionController::bind_max_value(float val, float max_val) {
  if (val > max_val)
    return max_val;
  else if (val < -max_val)
    return -max_val;
  return val;
}

void PositionController::update_quaternion() {
  float cr2 = cosf(roll_target_ * M_PI / 180.0f * 0.5f);
  float cp2 = cosf(pitch_target_ * M_PI / 180.0f * 0.5f);
  float cy2 = cosf(yaw_target_ * M_PI / 180.0f * 0.5f);
  float sr2 = sinf(roll_target_ * M_PI / 180.0f * 0.5f);
  float sp2 = sinf(pitch_target_ * M_PI / 180.0f * 0.5f);
  float sy2 = sinf(yaw_target_ * M_PI / 180.0f * 0.5f);

  q1_ = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
  q2_ = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
  q3_ = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
  q4_ = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;
}

PositionController::PositionController() {
  set_max_vel(MAX_HORZ_VEL);

  vel_pgain.x = .9;
  vel_pgain.y = .9;
  vel_pgain.z = .9;
}

void PositionController::set_pos_pgain(float gain) {
  pos_pgain.x = gain;
  pos_pgain.y = gain;
  pos_pgain.z = gain;
}
void PositionController::set_vel_pgain(float gain) {
  vel_pgain.x = gain;
  vel_pgain.y = gain;
  vel_pgain.z = gain;
}
void PositionController::set_vel_igain(float gain) {
  vel_igain.x = gain;
  vel_igain.y = gain;
  vel_igain.z = gain;
}
void PositionController::set_max_vel(float max_velocity) {
  max_vel.x = max_velocity;
  max_vel.y = max_velocity;
  max_vel.z = max_velocity;
}

PositionControllerState PositionController::run_loop(Vector3f current_pos,
                                                     Vector3f desired_pos) {
  update_dt();

  pos_curr = current_pos;

  // updating current velocity
  update_velocity_state();
  calc_average_velocity();
  calc_average_position();

  // Calculating position error
  pos_desi = desired_pos;
  pos_err = pos_desi - pos_curr_avg;

  // Calculating desired velocity
  vel_desi = pos_err.multiplyGain(pos_pgain);

  // bind velocity to max
  vel_desi.bindToMaxVal(max_vel);

  // calculating velocity error
  vel_err = vel_desi - vel_curr;

  // updating velocity error integration
  update_velocity_err_integration();

  // calculating desired acceleration
  acc_desi = vel_err.multiplyGain(vel_pgain) + vel_int.multiplyGain(vel_igain);

  reinitialize_state = false;
  return {pos_curr, vel_curr, vel_desi, acc_desi};
}

Vector3f PositionController::acceleration_to_attitude(float forward_acc,
                                                      float right_acc,
                                                      float rot,
                                                      float desir_yaw) {
  forward_acc_ = forward_acc;
  right_acc_ = right_acc;
  
  rot_ = rot * M_PI / 180.0f;

  rot_right_acc_ = right_acc * cos(rot_) - forward_acc * sin(rot_);
  rot_forward_acc_ = right_acc * sin(rot_) + forward_acc * cos(rot_);

  pitch_target_ = atanf(-rot_forward_acc_ / (9.806)) * (180.0f / M_PI);
  float cos_pitch_target = cosf(pitch_target_ * M_PI / 180.0f);
  roll_target_ =
      atanf(rot_right_acc_ * cos_pitch_target / (9.806)) * (180.0f / M_PI);

  pitch_target_ = bind_max_value(pitch_target_, MAX_ANGLE);
  roll_target_ = bind_max_value(roll_target_, MAX_ANGLE);
  yaw_target_ = desir_yaw;

  return {roll_target_, pitch_target_, yaw_target_};
}

string PositionController::get_state_string() {
  stringstream output;
  output << std::fixed;
  output << std::setprecision(5);

  output << pos_curr.x << ',' << pos_curr.y << ',' << pos_curr.z << ',';
  output << pos_last.x << ',' << pos_last.y << ',' << pos_last.z << ',';
  output << pos_err.x << ',' << pos_err.y << ',' << pos_err.z << ',';
  output << pos_desi.x << ',' << pos_desi.y << ',' << pos_desi.z << ',';
  output << vel_curr.x << ',' << vel_curr.y << ',' << vel_curr.z << ',';
  output << vel_desi.x << ',' << vel_desi.y << ',' << vel_desi.z << ',';
  output << vel_err.x << ',' << vel_err.y << ',' << vel_err.z << ',';
  output << vel_int.x << ',' << vel_int.y << ',' << vel_int.z << ',';
  output << acc_desi.x << ',' << acc_desi.y << ',' << acc_desi.z << ',';

  output << forward_acc_ << ',' << right_acc_ << ',';
  output << rot_forward_acc_ << ',' << rot_right_acc_ << ',';

  output << pitch_target_ << ',';
  output << roll_target_ << ',';
  output << yaw_target_;
  return output.str();
}
