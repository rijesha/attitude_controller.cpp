#include "measurementpredictor.h"

MeasurementPredictor::MeasurementPredictor() {}

void MeasurementPredictor::initialize(const DataPointType sensor_type)
{

  this->current_type = sensor_type;

  this->nz = NZ_LIDAR;
  this->R = MatrixXd(this->nz, this->nz);
  this->R << VAR_PX, 0,
      0, VAR_PY;
}

MatrixXd MeasurementPredictor::compute_sigma_z(const MatrixXd &sigma_x)
{

  const double THRESH = 1e-4;
  MatrixXd sigma = MatrixXd::Zero(this->nz, NSIGMA);

  for (int c = 0; c < NSIGMA; c++)
  {
    sigma(0, c) = sigma_x(0, c); // px
    sigma(1, c) = sigma_x(1, c); // py
  }

  return sigma;
}

MatrixXd MeasurementPredictor::compute_z(const MatrixXd &sigma)
{

  VectorXd z = VectorXd::Zero(this->nz);

  for (int c = 0; c < NSIGMA; c++)
  {
    z += WEIGHTS[c] * sigma.col(c);
  }

  return z;
}

MatrixXd MeasurementPredictor::compute_S(const MatrixXd &sigma, const MatrixXd &z)
{

  VectorXd dz;
  MatrixXd S = MatrixXd::Zero(this->nz, this->nz);

  for (int c = 0; c < NSIGMA; c++)
  {

    dz = sigma.col(c) - z;

    S += WEIGHTS[c] * dz * dz.transpose();
  }

  S += this->R;
  return S;
}

void MeasurementPredictor::process(const MatrixXd &sigma_x, const DataPointType sensor_type)
{

  // let the MeasurementPredictor know whether it's processing a LIDAR or RADAR measurement
  this->initialize(sensor_type);
  // transform predicted sigma_x into measurement space
  this->sigma_z = this->compute_sigma_z(sigma_x);
  // get the mean predicted measurement vector z
  this->z = this->compute_z(this->sigma_z);
  // get the measurement covariance matrix S
  this->S = this->compute_S(this->sigma_z, this->z);
}

VectorXd MeasurementPredictor::getz() const
{
  return this->z;
}

MatrixXd MeasurementPredictor::getS() const
{
  return this->S;
}

MatrixXd MeasurementPredictor::get_sigma() const
{
  return this->sigma_z;
}
