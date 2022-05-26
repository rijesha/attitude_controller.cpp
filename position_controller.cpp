#include "position_controller.h"

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
  timestamp = (current_run_time - start_time).count();
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

PositionController::PositionController(float max_angle, float max_speed)
    : max_vel{max_speed, max_speed, max_speed}, max_angle_(max_angle) {}

float PositionController::get_rate(float current_value, float desired_value,
                                   float p_gain) {
  return (desired_value - current_value) * p_gain;
}

float PositionController::wrap_angle(float angle) {
  if (angle > 180) {
    return angle - 360;
  }
  return angle;
}

PositionControllerState PositionController::run_loop(Vector3f current_pos,
                                                     Vector3f desired_pos) {
  update_dt();

  DataPoint sensor_data;
  VectorXd lidar_vec(NZ_LIDAR);
  lidar_vec << current_pos.x, current_pos.y;
  cout << timestamp << endl;
  sensor_data.set(timestamp/1000, DataPointType::LIDAR, lidar_vec);

  VectorXd prediction;
  VectorXd measurement;
  DataPoint estimation;

  fusionUKF.process(sensor_data);
  prediction = fusionUKF.get();
  double nis = fusionUKF.get_nis();
  cout << "raw_x: " << current_pos.x << "raw_y: " << current_pos.y
       << " x: " << prediction[0] << " y: " << prediction[1]
       << " dx: " << prediction[2] << " dy: " << prediction[3] << endl;
  pos_curr.z = current_pos.z;
  pos_curr.x = prediction[0];
  pos_curr.y = prediction[1];

  vel_curr.x = prediction[2];
  vel_curr.y = prediction[3];

  // updating current velocity
  // update_velocity_state();
  // calc_average_velocity();
  // calc_average_position();

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
                                                      float rot) {
  forward_acc_ = forward_acc;
  right_acc_ = right_acc;

  rot_ = rot * M_PI / 180.0f;

  rot_right_acc_ = right_acc * cos(rot_) - forward_acc * sin(rot_);
  rot_forward_acc_ = right_acc * sin(rot_) + forward_acc * cos(rot_);

  pitch_target_ = atanf(-rot_forward_acc_ / (9.806)) * (180.0f / M_PI);
  float cos_pitch_target = cosf(pitch_target_ * M_PI / 180.0f);
  roll_target_ =
      atanf(rot_right_acc_ * cos_pitch_target / (9.806)) * (180.0f / M_PI);

  pitch_target_ = bind_max_value(pitch_target_, max_angle_);
  roll_target_ = bind_max_value(roll_target_, max_angle_);

  return {roll_target_, pitch_target_, 0};
}

float PositionController::get_yaw_rate(float current_yaw, float desired_yaw) {
  current_yaw = wrap_angle(current_yaw);
  desired_yaw = wrap_angle(desired_yaw);
  return get_rate(current_yaw, desired_yaw, 0.8);
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
