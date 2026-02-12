#ifndef STATE_H
#define STATE_H

#include "util.h"

#include <ISM6HG256XSensor.h>
#include <ArduinoEigen.h>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

// Position is has y perpendicular to the ground

class FlightState {
public:
  FlightState() {}

  void push_baro(double pressure, double temperature, Millis sample_rate) {}

  void push_imu(ISM6HG256X_Axes_t &acc, ISM6HG256X_Axes_t &gyro, Millis sample_rate) {}

  bool done() { return false; }
};

class RestState {
  // There is a degree of freedom (roll I believe) since this is based
  // On the accelerometer originally so y is perpendicular to the ground
  // After applying this rotation to a sampled accelerometer reading
  Eigen::Quaterniond rot = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  double acceleration;

  bool inited = false;

public:
  RestState() {}

  void push_baro(double pressure, double temperature, Millis sample_rate);

  void push_imu(ISM6HG256X_Axes_t &acc, ISM6HG256X_Axes_t &gyro, Millis sample_rate);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);
};

#endif

