#include <chrono>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "vector3.h"

#include "datapoint.h"
#include "fusionukf.h"

using namespace std;

#define RESET_STATE_DT 0.1

#define POS_AVGS 3
#define VEL_AVGS 5

struct PositionControllerState {
  Vector3f position;
  Vector3f velocity;
  Vector3f velocity_desired;
  Vector3f acceleration_desired;
};

class PositionController {
 private:
  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      last_run_time = std::chrono::high_resolution_clock::now();
  std::chrono::time_point<std::chrono::high_resolution_clock,
                          std::chrono::nanoseconds>
      current_run_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed = current_run_time - last_run_time;

  float dt = 0.1;

  int vel_avg_ind = 0;
  Vector3f vel_avg[VEL_AVGS] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  Vector3f vel_curr_avg = {0, 0, 0};

  int pos_avg_ind = 0;
  Vector3f pos_avg[POS_AVGS] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  Vector3f pos_curr_avg = {0, 0, 0};

  float get_rate(float current_value, float desired_value, float p_gain);
  float wrap_angle(float angle);

  float forward_acc_;
  float right_acc_;
  float rot_;
  float rot_right_acc_;
  float rot_forward_acc_;
  float pitch_target_;
  float roll_target_;
  float yaw_target_;

  void calc_average_position();
  void calc_average_velocity();
  void update_dt();
  void update_velocity_state();
  void update_velocity_err_integration();
  float bind_max_value(float val, float max_val);

  int pos_avg_i_, vel_avg_i_ = 0;

  Vector3f pos_curr = {0, 0, 0};
  Vector3f pos_last = {0, 0, 0};
  Vector3f pos_err = {0, 0, 0};

  Vector3f pos_desi = {0, 0, 0};

  Vector3f vel_curr = {0, 0, 0};
  Vector3f vel_last = {0, 0, 0};

  Vector3f vel_desi = {0, 0, 0};
  Vector3f vel_err = {0, 0, 0};
  Vector3f vel_int = {0, 0, 0};

  Vector3f acc_desi = {0, 0, 0};

  Vector3f pos_pgain = {0.1, 0.1, 0.1};
  Vector3f vel_pgain = {0.1, 0.1, 0.1};

  Vector3f vel_igain = {0.0, 0.0, 0.0};
  Vector3f max_vel;

  float yaw_gain{0.8};

  float max_angle_;


  FusionUKF fusionUKF;

 public:
  PositionController(float max_angle, float max_speed);

  PositionControllerState run_loop(Vector3f current_pos, Vector3f desired_pos);
  Vector3f acceleration_to_attitude(float forward_acc, float right_acc,
                                    float rot);
  float get_yaw_rate(float current_yaw, float desired_yaw);

  string get_state_string(void);

  bool reinitialize_state = true;

  void set_pos_pgain(float gain) {
    pos_pgain.x = gain;
    pos_pgain.y = gain;
  };
  void set_vel_pgain(float gain) {
    vel_pgain.x = gain;
    vel_pgain.y = gain;
    gain;
  };
  void set_vel_igain(float gain) {
    vel_igain.x = gain;
    vel_igain.y = gain;
  };

  void set_pos_z_pgain(float gain) { pos_pgain.z = gain; };
  void set_vel_z_pgain(float gain) { vel_pgain.z = gain; };
  void set_vel_z_igain(float gain) { vel_igain.z = gain; };
  void set_yaw_gain(float gain) { yaw_gain = gain; };

  float get_pos_pgain() { return pos_pgain.x; };
  float get_vel_pgain() { return vel_pgain.x; };
  float get_vel_igain() { return vel_igain.x; };
  float get_pos_z_pgain() { return pos_pgain.z; };
  float get_vel_z_pgain() { return vel_pgain.z; };
  float get_vel_z_igain() { return vel_igain.z; };
  float get_yaw_gain() { return yaw_gain; };
};
