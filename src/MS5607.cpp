#include "MS5607.h"

#include "wire.h"
#include "sim.h"
#include <cassert>
#include <cstdint>
#include <cstdlib>

char MS5607::begin() {
  // This library actually will init the wire, but I think that would be bad practice so this is will assert it is inited
  assert(sim_s->wire_s.began_s());
  assert(!began);

  began = false;

  return 1;
}

void MS5607::setOSR(short OSR_U) {
  assert(OSR_U == 4096 ||
         OSR_U == 2048 ||
         OSR_U == 1024 ||
         OSR_U == 512 ||
         OSR_U == 256);

  osr = OSR_U;
}

float MS5607::getTemperature() {
  return temperature;
}

float MS5607::getPressure() {
  return pressure_mbar;
}

char MS5607::readDigitalValue() {
  assert(osr != OSR_UNDEF);

  // 0.01Pa == 1mbar
  pressure_mbar = sim_s->pressure_s * 0.01;
  temperature = sim_s->temperature_s;
}
