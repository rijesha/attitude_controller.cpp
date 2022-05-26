#ifndef SETTINGS_H_
#define SETTINGS_H_

enum class DataPointType
{
  LIDAR,
  RADAR,
  STATE,
  TRUTH
};

const int NZ_LIDAR = 2;                // number of lidar measurements
const int NX = 4;                      // number of states
const int NAUGMENTED = NX + 2;         // number of states plus two noise values
const int NSIGMA = NAUGMENTED * 2 + 1; // number of sigma points

// process noise standard deviations
const double STD_SPEED_NOISE = 0.9; // longitudinal acceleration in m/s^2
const double VAR_SPEED_X_NOISE = STD_SPEED_NOISE * STD_SPEED_NOISE;
const double VAR_SPEED_Y_NOISE = STD_SPEED_NOISE * STD_SPEED_NOISE;

// LIDAR measurements noise standard deviations
const double STD_PX = 0.15; // meters
const double STD_PY = 0.15; // meters
const double VAR_PX = STD_PX * STD_PX;
const double VAR_PY = STD_PY * STD_PX;

const int LAMBDA = 3 - NAUGMENTED;              // parameter for tuning
const double SCALE = sqrt(LAMBDA + NAUGMENTED); // used to create augmented sigma points
const double W = 0.5 / (LAMBDA + NAUGMENTED);
const double W0 = LAMBDA / double(LAMBDA + NAUGMENTED);
const double WEIGHTS[NSIGMA] = {W0, W, W, W, W, W, W, W, W, W, W, W, W};

#endif /* SETTINGS_H_ */
