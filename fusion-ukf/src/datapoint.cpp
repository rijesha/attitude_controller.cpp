#include "datapoint.h"

using namespace std;

DataPoint::DataPoint()
{
  this->initialized = false;
}

DataPoint::DataPoint(const long long timestamp, const DataPointType &data_type, const VectorXd &raw)
{
  this->set(timestamp, data_type, raw);
}

void DataPoint::set(const long long timestamp, const DataPointType &data_type, const VectorXd &raw)
{
  this->timestamp = timestamp;
  this->data_type = data_type;
  this->raw = raw;
  this->initialized = true;
}

VectorXd DataPoint::get() const
{
  return this->raw;
}

VectorXd DataPoint::get_state() const
{

  VectorXd state(NX);

  double px = this->raw(0);
  double py = this->raw(1);
  state << px, py, 0.0, 0.0, 0.0;

  return state;
}

VectorXd DataPoint::get_vec() const
{

  VectorXd vec(NX - 1);

  if (this->data_type == DataPointType::LIDAR)
  {

    double px = this->raw(0);
    double py = this->raw(1);
    vec << px, py, 0.0, 0.0;
  }

  return vec;
}

long long DataPoint::get_timestamp() const
{
  return this->timestamp;
}

DataPointType DataPoint::get_type() const
{
  return this->data_type;
}

void DataPoint::print() const
{

  if (this->initialized)
  {

    cout << "Timestamp: " << this->timestamp << endl;
    cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2) " << endl;
    cout << "Raw Data: " << endl;
    cout << this->raw << endl;
  }
  else
  {

    cout << "DataPoint is not initialized." << endl;
  }
}
