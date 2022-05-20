#include "statepredictor.h"

StatePredictor::StatePredictor() {}

MatrixXd StatePredictor::compute_augmented_sigma(const VectorXd &current_x, const MatrixXd &current_P)
{

  MatrixXd augmented_sigma = MatrixXd::Zero(NAUGMENTED, NSIGMA);
  VectorXd augmented_x = VectorXd::Zero(NAUGMENTED);
  MatrixXd augmented_P = MatrixXd::Zero(NAUGMENTED, NAUGMENTED);

  augmented_x.head(NX) = current_x;

  augmented_P.topLeftCorner(NX, NX) = current_P;
  augmented_P(NX, NX) = VAR_SPEED_X_NOISE;
  augmented_P(NX + 1, NX + 1) = VAR_SPEED_Y_NOISE;

  const MatrixXd L = augmented_P.llt().matrixL();
  augmented_sigma.col(0) = augmented_x;

  for (int c = 0; c < NAUGMENTED; c++)
  {
    const int i = c + 1;
    augmented_sigma.col(i) = augmented_x + SCALE * L.col(c);
    augmented_sigma.col(i + NAUGMENTED) = augmented_x - SCALE * L.col(c);
  }

  return augmented_sigma;
}

MatrixXd StatePredictor::predict_sigma(const MatrixXd &augmented_sigma, double dt)
{

  const double THRESH = 0.001;
  MatrixXd predicted_sigma = MatrixXd(NX, NSIGMA);

  for (int c = 0; c < NSIGMA; ++c)
  {

    /*************************************
     * Get the current state
     *************************************/
    const double px = augmented_sigma(0, c);
    const double py = augmented_sigma(1, c);
    const double speed_x = augmented_sigma(2, c);
    const double speed_y = augmented_sigma(3, c);
    const double speed_x_noise = augmented_sigma(4, c);
    const double speed_y_noise = augmented_sigma(5, c);

    /*************************************
     * predict the next state with noise
     * USING THE CTRV MODEL
     *************************************/

    const double dt2 = dt * dt;

    // predicted position noise
    const double p_x_noise = 0.5 * speed_x_noise * dt2;
    const double p_y_noise = 0.5 * speed_y_noise * dt2;

    const double dspeed_x = speed_x * dt; // change in speed
    const double dspeed_y = speed_y * dt; // change in speed

    // predicted speed = assumed constant speed + noise
    const double p_speed_x = speed_x + speed_x_noise * dt;
    const double p_speed_y = speed_y + speed_y_noise * dt;

    // where predicted positions will be stored
    double p_px, p_py;

    // moving straight
    p_px = px + dspeed_x + p_x_noise;
    p_py = py + dspeed_y + p_y_noise;

    /*************************************
     * Write the prediction to the appropriate column
     *************************************/
    predicted_sigma(0, c) = p_px;
    predicted_sigma(1, c) = p_py;
    predicted_sigma(2, c) = p_speed_x;
    predicted_sigma(3, c) = p_speed_y;
  }

  return predicted_sigma;
}

VectorXd StatePredictor::predict_x(const MatrixXd &predicted_sigma)
{

  VectorXd predicted_x = VectorXd::Zero(NX);

  for (int c = 0; c < NSIGMA; c++)
  {
    predicted_x += WEIGHTS[c] * predicted_sigma.col(c);
  }

  return predicted_x;
}

MatrixXd StatePredictor::predict_P(const MatrixXd &predicted_sigma, const VectorXd &predicted_x)
{

  MatrixXd predicted_P = MatrixXd::Zero(NX, NX);
  VectorXd dx = VectorXd(NX);

  for (int c = 0; c < NSIGMA; c++)
  {

    dx = predicted_sigma.col(c) - predicted_x;
    dx(3) = normalize(dx(3));
    predicted_P += WEIGHTS[c] * dx * dx.transpose();
  }

  return predicted_P;
}

void StatePredictor::process(VectorXd &current_x, MatrixXd &current_P, double dt)
{

  MatrixXd augmented_sigma = compute_augmented_sigma(current_x, current_P);
  this->sigma = predict_sigma(augmented_sigma, dt);
  this->x = predict_x(this->sigma);
  this->P = predict_P(this->sigma, this->x);
}

MatrixXd StatePredictor::getP() const
{
  return this->P;
}

MatrixXd StatePredictor::get_sigma() const
{
  return this->sigma;
}

VectorXd StatePredictor::getx() const
{
  return this->x;
}
