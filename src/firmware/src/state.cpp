#include "state.h"
#include "util.h"

#include "logging.h"

void RestState::push_baro(double pressure, double temperature, Millis sample_rate) {
}

void RestState::push_imu(ISM6HG256X_Axes_t &acc, ISM6HG256X_Axes_t &gyro, Millis sample_rate) {
  // We want to find that q such that when we rotate an accelerometer reading by q
  // it gets transformed into the coordinate frame where y is up
  // We don't care about what the gyro says in the rest state sense we have a method to determine the absolute rotation
  // TODO: Take multiple samples to determine rotation (this depends on how we read the imu)

  Eigen::Vector3d acc_vec(acc.x, acc.y, acc.z);
  // This is the vector that gravity actually points when the board is facing up (the accelerometer is mounted at an angle)
  // We don't care about magnitude since it is a direction
  Eigen::Vector3d down(0.0, 0.0, 1.0);

  // The rotation that takes acc and turns it into down
  rot = Eigen::Quaterniond::FromTwoVectors(acc_vec, down);

  acceleration = acc_vec.norm();

  inited = true;
}

bool RestState::try_init_flying(FlightState &state) {
  if (!inited) {
    return false;
  }

  if (abs(acceleration - gravity_acc) > 30) {
    return true;
  }

  return false;
}

