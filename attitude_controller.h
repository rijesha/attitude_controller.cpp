#include "vector3.h"
#include <ctime>
#include <chrono>
#include <string>
#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

#define RESET_STATE_DT 0.1
#define MAX_HORZ_VEL 0.1
#define MAX_VERT_VEL 0.1
#define AVGS 3

class AttitudeController {
    private:
        bool reinitialize_state = true;

        Vector3f ned_pos_curr = {0, 0, 0};
        Vector3f ned_pos_last = {0, 0, 0};
        Vector3f ned_pos_err  = {0, 0, 0};

        Vector3f ned_pos_desi = {0, 0, 0};

        Vector3f ned_vel_curr = {0, 0, 0};
        Vector3f ned_vel_last = {0, 0, 0};

        Vector3f ned_vel_desi = {0, 0, 0};
        Vector3f ned_vel_err  = {0, 0, 0};
        Vector3f ned_vel_int  = {0, 0, 0};

        Vector3f ned_acc_desi = {0, 0, 0};

        Vector3f pos_pgain = {1, 1, 1};
        Vector3f vel_pgain = {1, 1, 1};
        Vector3f vel_igain = {0.0001, 0.0001, 0.0001};
        Vector3f max_vel = {MAX_HORZ_VEL, MAX_HORZ_VEL, MAX_VERT_VEL};

        float hover_throttle = 0.5;
        float pitch_target;
        float cos_pitch_target;
        float roll_target;   
        float yaw_target;
        float thrust;

        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> last_run_time = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock, std::chrono::nanoseconds> current_run_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_run_time - last_run_time;

        float dt = 0.1;

        int avg_ind = 0;
        Vector3f vel_avg[AVGS] = {{0, 0, 0},{0, 0, 0},{0, 0, 0}};
        Vector3f ned_vel_curr_avg = {0,0,0};

        void calc_average_velocity();
        void update_dt();
        void update_velocity_state();
        void update_velocity_err_integration();
        float bind_max_value(float val, float max_val);
        float bind_max_value(float val, float max_val, float min_val);

        void updateQuaternion();
        float q1,q2,q3,q4;

    public:
        AttitudeController();
        void run_loop(float curr_x, float curr_y, float curr_z, float desir_x, float desir_y, float desir_z, float desir_yaw);
        string get_state_string(void );
}