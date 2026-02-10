#ifndef STATE_H
#define STATE_H

#include <ISM6HG256XSensor.h>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

class FlightState {
public:
  void push_baro(double pressure, double temperature) {}

  void push_imu(ISM6HG256X_Axes_t &imu, ISM6HG256X_Axes_t &gyro) {}

  bool done() { return false; }
};

class RestState {
public:
  void push_baro(double pressure, double temperature) {}

  void push_imu(ISM6HG256X_Axes_t &imu, ISM6HG256X_Axes_t &gyro) {}

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state) { return false; }
};

#endif

