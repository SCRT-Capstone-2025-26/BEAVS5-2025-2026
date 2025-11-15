#ifndef SIM_H
#define SIM_H

#include "board.h"
#include "misc.h"
#include <boost/coroutine2/all.hpp>
#include <functional>

const int pin_count_s = 30;

class Sim_s {
  unsigned long long micros0 = 0;
  unsigned long long micros1 = 0;

  boost::coroutines2::coroutine<void>::push_type *yield0 = nullptr;
  boost::coroutines2::coroutine<void>::push_type *yield1 = nullptr;

  boost::coroutines2::coroutine<void>::pull_type cpu0;
  boost::coroutines2::coroutine<void>::pull_type cpu1;

  int cpu = 0;

  void yield();
  void run0(boost::coroutines2::coroutine<void>::push_type &yield);
  void run1(boost::coroutines2::coroutine<void>::push_type &yield);

public:
  // Maybe move to sim.cpp
  Sim_s()
      : cpu0(std::bind(&Sim_s::run0, this, std::placeholders::_1)),
        cpu1(std::bind(&Sim_s::run1, this, std::placeholders::_1)),
        board_s(this) {
    board_s.bmp.sim_s = this;
  };

  unsigned long millis();
  unsigned long micros();
  void pinMode(uint8_t, uint8_t);
  void digitalWrite(uint8_t, uint8_t);
  int digitalRead(uint8_t);
  void analogWrite(int, int);
  void delay(unsigned long);
  void delayMicroseconds(unsigned long);

  Board board_s;

  // https://docs.arduino.cc/language-reference/en/functions/analog-io/analogWriteResolution/
  // Arduino seemingly uses a global variable for the resolution of the analog writes for compatibility
  // It is defaulted to 8. This is different to the analog read default of 10 bits (although unused in this code)
  // TODO: Restrictions on this number maybe
  int analog_res_s = 8;

  unsigned long long micros_s = 0;
  Pin_s pins_s[pin_count_s] = {};
  Pin_s analog_pins_s[pin_count_s] = {};

  void step();
  void step_to(unsigned long long time);
  void step_by(unsigned long long time);
};

#endif
