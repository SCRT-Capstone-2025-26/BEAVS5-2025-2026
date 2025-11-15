#include "bmp.h"

#include "wire.h"
#include "sim.h"
#include <cassert>
#include <cstdint>
#include <cstdlib>

bool Adafruit_BMP3XX::begin_I2C(uint8_t addr, TwoWire *theWire) {
  assert(!began);
  assert(addr == 0x77);
  assert(theWire->began_s());

  // This should be inited by sim when board is created
  assert(sim_s != nullptr);

  began = true;
  return true;
}

float Adafruit_BMP3XX::readPressure(void) {
  assert(began);

  performReading();

  return pressure;
}

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t os) {
  assert(began);
  assert(os <= BMP3_OVERSAMPLING_32X);

  temp_os = os;
  return true;
}

bool Adafruit_BMP3XX::setPressureOversampling(uint8_t os) {
  assert(began);
  assert(os <= BMP3_OVERSAMPLING_32X);

  pres_os = os;
  return true;
}

bool Adafruit_BMP3XX::performReading() {
  assert(began);
  // These are the recommended values
  assert((temp_os == BMP3_NO_OVERSAMPLING && pres_os <= BMP3_OVERSAMPLING_8X) ||
         (temp_os == BMP3_OVERSAMPLING_2X && pres_os > BMP3_OVERSAMPLING_8X));

  // These are the delays that reading in forced mode causes
  // These are the typical values not the max
  // This switch to too big tbh
  switch (pres_os) {
    case BMP3_NO_OVERSAMPLING:
      sim_s->delayMicroseconds(4820);
      break;
    case BMP3_OVERSAMPLING_2X:
      sim_s->delayMicroseconds(6840);
      break;
    case BMP3_OVERSAMPLING_4X:
      sim_s->delayMicroseconds(10880);
      break;
    case BMP3_OVERSAMPLING_8X:
      sim_s->delayMicroseconds(18690);
      break;
    case BMP3_OVERSAMPLING_16X:
      sim_s->delayMicroseconds(37140);
      break;
    case BMP3_OVERSAMPLING_32X:
      sim_s->delayMicroseconds(69460);
      break;
  }

  // Pressure is normally calibrated based on the board, but the raw value is read in 24 bits
  // So this just uses the min and max to approximate what the noise will look like
  // The board ranges from about 300hPa to 1100hPa and I am assuming all 24 bits are used
  // with 0 being min and 2^24 - 1 being the max the 0.5 is for rounding
  int64_t raw = (int64_t)(((pressure_s - 30000) / 80000 * (1 << 24)) + 0.5);
  if (raw >= (1 << 24)) { raw = (1 << 24) - 1; }
  if (raw < 0) { raw = 0; }
  
  // Add noise according to the bit accuarcy table
  uint8_t bits = 16 + pres_os;
  uint64_t mask = (1 << bits) - 1;

  int64_t observed = (raw & ~mask) | (rand() & mask);

  pressure = (((float)observed) / (1 << 24) * 80000) + 30000;

  return true;
}
