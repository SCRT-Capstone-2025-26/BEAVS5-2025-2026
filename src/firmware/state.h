#ifndef STATE_H
#define STATE_H

#include <ISM6HG256XSensor.h>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

class FlightState {
  void push_baro(double pressure, double temperature);

  void push_acc(ISM6HG256X_Axes_t &acc, ISM6HG256X_Axes_t &gyro);

  void done();
};

class RestState {
  void push_baro(double pressure, double temperature);

  void push_acc(ISM6HG256X_Axes_t &acc, ISM6HG256X_Axes_t &gyro);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);
};

#endif

